// Copyright 2026 isaac_ros_imx219 contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <cuda.h>
#include <cudaEGL.h>
#include <cuda_runtime.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "camera_info_manager/camera_info_manager.hpp"
#include "cuda_buffer/cuda_buffer_api.hpp"
#include "isaac_ros_imx219/convert.hpp"
#include "isaac_ros_imx219/frame_watchdog.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "isaac_ros_imx219/camera_node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "foxglove_msgs/msg/compressed_video.hpp"
#include "isaac_ros_imx219/native_encoder.hpp"

namespace imx219
{

// Keep public compressed streams on the standard DDS writer. On Lyrical, a
// departed buffer-aware reader can otherwise leave a Jazzy bridge on a silent
// channel. Encoded packets are already CPU bytes on both encoder paths.
template<typename Message>
void publish_encoded_packet(rclcpp::Publisher<Message> & publisher, const Message & message)
{
  rclcpp::Serialization<Message> serializer;
  rclcpp::SerializedMessage packet;
  serializer.serialize_message(&message, &packet);
  if (packet.size() <= message.data.size() || packet.size() > packet.capacity()) {
    throw std::runtime_error("Invalid CDR size for compressed video packet");
  }
  publisher.publish(packet);
}

void check_cu(CUresult code, const char * operation)
{
  if (code == CUDA_SUCCESS) {
    return;
  }
  const char * description = nullptr;
  cuGetErrorString(code, &description);
  throw std::runtime_error(std::string(operation) + ": " +
          (description ? description : "unknown CUDA driver error"));
}

void check_cuda(cudaError_t code, const char * operation)
{
  if (code != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(code));
  }
}

void check_argus(Argus::Status status, const char * operation)
{
  if (status != Argus::STATUS_OK) {
    throw std::runtime_error(std::string(operation) + ": Argus status " +
            std::to_string(status));
  }
}

template<typename Interface, typename Object>
Interface * require_interface(Object * object, const char * name)
{
  auto * interface = Argus::interface_cast<Interface>(object);
  if (!interface) {
    throw std::runtime_error(std::string("Missing Argus interface: ") + name);
  }
  return interface;
}

int64_t clock_ns(clockid_t clock)
{
  timespec value{};
  if (clock_gettime(clock, &value) != 0) {
    throw std::runtime_error("clock_gettime failed");
  }
  return static_cast<int64_t>(value.tv_sec) * 1000000000LL + value.tv_nsec;
}

// Owns the external capture resources. Destruction occurs on the capture thread,
// with the same CUDA primary context used by cuda_buffer.
class CaptureResources
{
public:
  Argus::UniqueObj<Argus::CameraProvider> provider;
  Argus::UniqueObj<Argus::CaptureSession> session;
  Argus::UniqueObj<Argus::OutputStream> output;
  Argus::UniqueObj<Argus::Request> request;
  Argus::ICaptureSession * capture{};
  Argus::IEGLOutputStream * egl_output{};
  CUeglStreamConnection connection{};
  cudaStream_t stream{};
  bool connected{};
  bool repeating{};
  std::unordered_map<CUarray, CUsurfObject> surfaces;

  ~CaptureResources()
  {
    if (repeating && capture) {
      capture->stopRepeat();
    }
    if (stream) {
      cudaStreamSynchronize(stream);  // Shutdown only; never in the frame loop.
    }
    if (capture) {
      capture->waitForIdle(3000000000ULL);
    }
    if (egl_output) {
      egl_output->disconnect();
    }
    if (connected) {
      cuEGLStreamConsumerDisconnect(&connection);
    }
    for (const auto & entry : surfaces) {
      cuSurfObjectDestroy(entry.second);
    }
    request.reset();
    output.reset();
    session.reset();
    provider.reset();
    if (stream) {
      cudaStreamDestroy(stream);
    }
  }

  CUsurfObject surface(CUarray array)
  {
    auto found = surfaces.find(array);
    if (found != surfaces.end()) {
      return found->second;
    }
    // Argus recycles a finite pool of EGL images. Bound the cache even on failure.
    if (surfaces.size() >= 128) {
      throw std::runtime_error("Unexpected unbounded Argus image pool");
    }
    CUDA_RESOURCE_DESC descriptor{};
    descriptor.resType = CU_RESOURCE_TYPE_ARRAY;
    descriptor.res.array.hArray = array;
    CUsurfObject handle{};
    check_cu(cuSurfObjectCreate(&handle, &descriptor), "cuSurfObjectCreate");
    surfaces.emplace(array, handle);
    return handle;
  }
};

class FrameLease
{
public:
  explicit FrameLease(CaptureResources & resources)
  : resources_(resources) {}

  ~FrameLease()
  {
    if (resource) {
      // Queue the release AFTER the conversion kernel on the same stream.
      // The producer must not recycle its input while CUDA is still reading it.
      cuEGLStreamConsumerReleaseFrame(&resources_.connection, resource, &resources_.stream);
    }
  }

  CUgraphicsResource resource{};

private:
  CaptureResources & resources_;
};

Yuv420 describe_frame(const CUeglFrame & frame, CaptureResources & resources)
{
  Yuv420 input;
  switch (frame.eglColorFormat) {
    case CU_EGL_COLOR_FORMAT_YUV420_PLANAR:
      break;
    case CU_EGL_COLOR_FORMAT_YVU420_PLANAR:
      input.swap_uv = true;
      break;
    case CU_EGL_COLOR_FORMAT_YUV420_PLANAR_ER:
      input.full_range = true;
      break;
    case CU_EGL_COLOR_FORMAT_YVU420_PLANAR_ER:
      input.full_range = true;
      input.swap_uv = true;
      break;
    case CU_EGL_COLOR_FORMAT_YUV420_PLANAR_709:
      input.matrix = 1;
      break;
    case CU_EGL_COLOR_FORMAT_YUV420_PLANAR_2020:
      input.matrix = 2;
      break;
    case CU_EGL_COLOR_FORMAT_YUV420_SEMIPLANAR:
      input.semiplanar = true;
      input.swap_uv = true;
      break;
    case CU_EGL_COLOR_FORMAT_YVU420_SEMIPLANAR:
      input.semiplanar = true;
      break;
    case CU_EGL_COLOR_FORMAT_YUV420_SEMIPLANAR_ER:
      input.semiplanar = true;
      input.full_range = true;
      input.swap_uv = true;
      break;
    case CU_EGL_COLOR_FORMAT_YVU420_SEMIPLANAR_ER:
      // CUDA EGL names describe component order, not byte order. 0x2c
      // is full-range Y with interleaved U,V bytes (see cudaEGL.h).
      input.semiplanar = true;
      input.full_range = true;
      break;
    case CU_EGL_COLOR_FORMAT_YUV420_SEMIPLANAR_709:
      input.semiplanar = true;
      input.matrix = 1;
      break;
    case CU_EGL_COLOR_FORMAT_YUV420_SEMIPLANAR_2020:
      input.semiplanar = true;
      input.matrix = 2;
      break;
    default:
      throw std::runtime_error("Unsupported EGL color format " +
              std::to_string(frame.eglColorFormat));
  }
  if (frame.cuFormat != CU_AD_FORMAT_UNSIGNED_INT8 ||
    frame.planeCount != (input.semiplanar ? 2U : 3U))
  {
    throw std::runtime_error("Expected 8-bit YUV420 planes from Argus ISP");
  }
  if (frame.frameType == CU_EGL_FRAME_TYPE_ARRAY) {
    for (unsigned plane = 0; plane < frame.planeCount; ++plane) {
      CUDA_ARRAY_DESCRIPTOR descriptor{};
      check_cu(cuArrayGetDescriptor(&descriptor, frame.frame.pArray[plane]), "array descriptor");
      const unsigned channels = (plane == 1 && input.semiplanar) ? 2 : 1;
      const unsigned width = plane == 0 ? frame.width : frame.width / 2;
      const unsigned height = plane == 0 ? frame.height : frame.height / 2;
      if (descriptor.NumChannels != channels || descriptor.Width != width ||
        descriptor.Height != height || descriptor.Format != CU_AD_FORMAT_UNSIGNED_INT8)
      {
        throw std::runtime_error("Unexpected CUDA array plane layout");
      }
    }
    input.y = resources.surface(frame.frame.pArray[0]);
    input.u = resources.surface(frame.frame.pArray[1]);
    if (!input.semiplanar) {
      input.v = resources.surface(frame.frame.pArray[2]);
    }
  } else {
    // Do not guess separate chroma pitch: CUeglFrame exposes only one pitch.
    // The tested Jetson Argus path is array-backed; add pitch support only with
    // per-plane stride evidence from the capture producer.
    throw std::runtime_error("Pitch-backed EGL frame unsupported; expected Argus CUDA arrays");
  }
  return input;
}

class Imx219Node : public rclcpp::Node
{
public:
  explicit Imx219Node(const rclcpp::NodeOptions & options)
  : Node("imx219", options)
  {
    sensor_id_ = parameter<int>("sensor_id", 0);
    sensor_mode_ = parameter<int>("sensor_mode", 2);
    width_ = parameter<int>("width", 1280);
    height_ = parameter<int>("height", 720);
    rotation_ = parameter<int>("rotation", 180);
    if (rotation_ != 0 && rotation_ != 180) {
      throw std::invalid_argument("rotation must be 0 or 180 degrees");
    }
    fps_ = parameter<double>("fps", 30.0);
    max_frames_ = parameter<int>("max_frames", 0);
    const auto retention_ms = parameter<int>("ipc_retention_ms", 1000);
    if (retention_ms < 0 || retention_ms > 5000) {
      throw std::invalid_argument("ipc_retention_ms must be in [0, 5000]");
    }
    stream_mode_ = parameter<std::string>("stream.mode", "off");
    const bool publish_compressed_image = parameter<bool>("stream.publish_compressed_image", false);
    stream_fps_ = parameter<int>("stream.fps", 15);
    stream_width_ = parameter<int>("stream.width", 960);
    bitrate_ = parameter<int>("stream.bitrate_kbps", 1500);
    interval_ = parameter<int>("stream.iframe_interval", 15);
    if ((stream_mode_ != "off" && stream_mode_ != "native_nvenc" && stream_mode_ != "isaac_ros_h264") ||
      stream_fps_ < 1 || stream_fps_ > 30 || bitrate_ < 300 || bitrate_ > 8000 ||
      interval_ < 1 || interval_ > 120 ||
      (stream_width_ != 640 && stream_width_ != 960 && stream_width_ != 1280)) {
      throw std::invalid_argument("Invalid stream settings");
    }
    max_exposure_us_ = parameter<int>("max_exposure_us", 0);
    exposure_us_ = parameter<int>("exposure_us", 0);
    if (exposure_us_ < 0 || (exposure_us_ > 0 && max_exposure_us_ > 0)) {
      throw std::invalid_argument("Use exposure_us or max_exposure_us, not both");
    }
    awb_mode_ = parameter<std::string>("awb_mode", "indoor");
    exposure_ev_ = parameter<double>("exposure_compensation_ev", -0.7);
    gain_r_ = parameter<double>("color_gain_r", 1.1772151898734178);
    gain_g_ = parameter<double>("color_gain_g", 1.0);
    gain_b_ = parameter<double>("color_gain_b", 0.8691588785046729);
    if (awb_mode_ != "auto" && awb_mode_ != "indoor" && awb_mode_ != "daylight") {
      throw std::invalid_argument("awb_mode must be auto, indoor or daylight");
    }
    for (auto gain : {gain_r_, gain_g_, gain_b_}) {
      if (!std::isfinite(gain) || gain < 0.25 || gain > 4.0) {
        throw std::invalid_argument("Color gains must be finite and in [0.25, 4]");
      }
    }
    if (!std::isfinite(exposure_ev_) || exposure_ev_ < -2.0 || exposure_ev_ > 2.0) {
      throw std::invalid_argument("exposure_compensation_ev must be in [-2, 2]");
    }
    frame_id_ = parameter<std::string>("frame_id", "imx219_optical_frame");
    const auto timestamp_clock = parameter<std::string>("timestamp_clock", "monotonic");
    auto camera_name = parameter<std::string>("camera_name", "");
    if (camera_name.empty()) {
      camera_name = "imx219_mode" + std::to_string(sensor_mode_) + "_" +
        std::to_string(width_) + "x" + std::to_string(height_) +
        "_rot" + std::to_string(rotation_);
    }
    auto camera_url = parameter<std::string>("camera_info_url", "auto");
    const bool measured_geometry = sensor_id_ == 0 && sensor_mode_ == 2 &&
      width_ == 1280 && height_ == 720 && rotation_ == 180;
    const std::string measured_file = "imx219_mode2_1280x720_rot180.yaml";
    if (camera_url == "auto") {
      camera_url = measured_geometry ?
        "package://isaac_ros_imx219/config/" + measured_file : "";
    }
    if (!measured_geometry && camera_url.find(measured_file) != std::string::npos) {
      throw std::invalid_argument(
              "Bundled calibration requires sensor 0, mode 2, 1280x720, rotation 180");
    }
    const auto qos_depth = parameter<int>("qos_depth", 3);
    if (get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument("A physical camera requires use_sim_time=false");
    }
    if (width_ <= 0 || height_ <= 0 || width_ % 2 || height_ % 2 ||
      width_ > 8192 || height_ > 8192 || !std::isfinite(fps_) || fps_ <= 0.0 || fps_ > 1000.0 ||
      sensor_id_ < 0 || sensor_mode_ < 0 || max_frames_ < 0 || max_exposure_us_ < 0 ||
      qos_depth < 1 || qos_depth > 32 || frame_id_.empty())
    {
      throw std::invalid_argument(
          "Invalid camera dimensions, rate, index, frame_id or queue depth");
    }
    if (timestamp_clock == "monotonic") {
      sensor_clock_ = CLOCK_MONOTONIC;
    } else if (timestamp_clock == "monotonic_raw") {
      sensor_clock_ = CLOCK_MONOTONIC_RAW;
    } else {
      throw std::invalid_argument("timestamp_clock must be monotonic or monotonic_raw");
    }
    retention_frames_ = static_cast<size_t>(std::ceil(fps_ * retention_ms / 1000.0)) + 1;
    if (retention_frames_ * width_ * height_ * 3ULL > 512ULL * 1024 * 1024) {
      throw std::invalid_argument("IPC retention exceeds 512 MiB; reduce ipc_retention_ms");
    }
    camera_info_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name);
    if (!camera_url.empty() && !camera_info_->loadCameraInfo(camera_url)) {
      throw std::runtime_error("Failed to load camera_info_url: " + camera_url);
    }
    if (!camera_info_->isCalibrated()) {
      RCLCPP_WARN(get_logger(),
          "Uncalibrated camera: publishing K=0; calibrate before metric pose");
    } else {
      const auto info = camera_info_->getCameraInfo();
      if (info.width != static_cast<uint32_t>(width_) ||
        info.height != static_cast<uint32_t>(height_))
      {
        throw std::invalid_argument("Calibration dimensions do not match capture");
      }
      RCLCPP_INFO(get_logger(), "Loaded calibration: %s; fx=%.6f fy=%.6f",
        camera_url.c_str(), info.k[0], info.k[4]);
    }
    // Fail before buffer-aware publisher construction: middleware cleanup can
    // mask a CUDA backend initialization error with a secondary crash.
    check_cu(cuInit(0), "CUDA initialization; check container GPU access");
    CUdevice device{};
    check_cu(cuDeviceGet(&device, 0), "CUDA device 0; check container GPU access");
    const auto qos = rclcpp::SensorDataQoS().keep_last(qos_depth);
    // External Isaac ROS consumers need the original allocation alive while importing CUDA IPC.
    rclcpp::PublisherOptions external;
    external.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", qos, external);
    if (stream_mode_ != "off") {
      video_pub_ = create_publisher<Video>("video/h264", rclcpp::SensorDataQoS(), external);
    }
    if (stream_mode_ == "isaac_ros_h264") {
      encoder_input_ = create_publisher<sensor_msgs::msg::Image>("internal/encode_input", rclcpp::QoS(2).reliable());
      if (publish_compressed_image) {
        compressed_pub_ = create_publisher<Compressed>("image_compressed", rclcpp::SensorDataQoS(), external);
      }
      encoded_sub_ = create_subscription<Compressed>("internal/encoded", rclcpp::SensorDataQoS(),
        [this](Compressed::UniquePtr encoded) {
          if (encoded->data.get_backend_type() == "cuda") {
            // Both public topics export standard CDR containing only encoded bytes.
            encoded->data = rosidl::Buffer<uint8_t>(encoded->data.to_vector());
          }
          if (compressed_pub_ && compressed_pub_->get_subscription_count()) {
            publish_encoded_packet(*compressed_pub_, *encoded);
          }
          if (video_pub_->get_subscription_count()) {
            Video msg;
            msg.timestamp = encoded->header.stamp;
            msg.frame_id = encoded->header.frame_id;
            msg.format = "h264";
            msg.data = std::move(encoded->data);
            publish_encoded_packet(*video_pub_, msg);
          }
        });
    }
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);
    shutdown_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
          if (done_) {
            rclcpp::shutdown(get_node_base_interface()->get_context());
          }
      });
    worker_ = std::thread([this]() {
          try {
            capture();
          } catch (const std::exception & error) {
            failed_ = true;
            RCLCPP_FATAL(get_logger(), "Capture failed: %s", error.what());
          }
          done_ = true;
      });
  }

  ~Imx219Node() override
  {
    stopping_ = true;
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  bool failed() const {return failed_;}

private:
  template<typename T>
  T parameter(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }

  void capture()
  {
    check_cuda(cudaSetDevice(0), "cudaSetDevice");
    // Runtime API initializes/uses the primary context, shared with cuda_buffer.
    check_cuda(cudaFree(nullptr), "initialize primary context");
    CaptureResources resources;
    check_cuda(cudaStreamCreateWithFlags(&resources.stream, cudaStreamNonBlocking),
        "create stream");
    resources.provider.reset(Argus::CameraProvider::create());
    auto * provider = require_interface<Argus::ICameraProvider>(
      resources.provider.get(), "ICameraProvider");
    std::vector<Argus::CameraDevice *> devices;
    check_argus(provider->getCameraDevices(&devices), "getCameraDevices");
    if (static_cast<size_t>(sensor_id_) >= devices.size()) {
      throw std::runtime_error(
          "sensor_id not enumerated by Argus; check driver and CSI connection");
    }
    auto * properties = require_interface<Argus::ICameraProperties>(
      devices[sensor_id_], "ICameraProperties");
    std::vector<Argus::SensorMode *> modes;
    check_argus(properties->getAllSensorModes(&modes), "getAllSensorModes");
    for (size_t i = 0; i < modes.size(); ++i) {
      auto * mode = require_interface<Argus::ISensorMode>(modes[i], "ISensorMode");
      const auto size = mode->getResolution();
      RCLCPP_INFO(get_logger(), "Sensor mode %zu: %ux%u max %.2f fps", i,
        size.width(), size.height(), 1e9 / mode->getFrameDurationRange().min());
    }
    if (static_cast<size_t>(sensor_mode_) >= modes.size()) {
      throw std::runtime_error("sensor_mode out of range");
    }
    auto * mode = require_interface<Argus::ISensorMode>(modes[sensor_mode_], "ISensorMode");
    const uint64_t frame_ns = static_cast<uint64_t>(std::llround(1e9 / fps_));
    if (frame_ns + 1000 < mode->getFrameDurationRange().min() ||
      frame_ns > mode->getFrameDurationRange().max() ||
      static_cast<unsigned>(width_) > mode->getResolution().width() ||
      static_cast<unsigned>(height_) > mode->getResolution().height())
    {
      throw std::runtime_error("Requested output or fps exceeds selected sensor mode");
    }
    resources.session.reset(provider->createCaptureSession(devices[sensor_id_]));
    resources.capture = require_interface<Argus::ICaptureSession>(
      resources.session.get(), "ICaptureSession");
    Argus::UniqueObj<Argus::OutputStreamSettings> settings(
      resources.capture->createOutputStreamSettings(Argus::STREAM_TYPE_EGL));
    auto * egl_settings = require_interface<Argus::IEGLOutputStreamSettings>(
      settings.get(), "IEGLOutputStreamSettings");
    check_argus(egl_settings->setPixelFormat(Argus::PIXEL_FMT_YCbCr_420_888), "setPixelFormat");
    check_argus(egl_settings->setResolution(Argus::Size2D<uint32_t>(width_, height_)),
        "resolution");
    check_argus(egl_settings->setMode(Argus::EGL_STREAM_MODE_MAILBOX), "mailbox mode");
    check_argus(egl_settings->setMetadataEnable(true), "metadata enable");
    resources.output.reset(resources.capture->createOutputStream(settings.get()));
    resources.egl_output = require_interface<Argus::IEGLOutputStream>(
      resources.output.get(), "IEGLOutputStream");
    check_cu(cuEGLStreamConsumerConnect(
        &resources.connection, resources.egl_output->getEGLStream()), "connect CUDA consumer");
    resources.connected = true;
    resources.request.reset(resources.capture->createRequest(Argus::CAPTURE_INTENT_VIDEO_RECORD));
    auto * request = require_interface<Argus::IRequest>(resources.request.get(), "IRequest");
    check_argus(request->enableOutputStream(resources.output.get()), "enableOutputStream");
    auto * controls = require_interface<Argus::IAutoControlSettings>(
      request->getAutoControlSettings(), "IAutoControlSettings");
    const auto awb = awb_mode_ == "indoor" ? Argus::AWB_MODE_INCANDESCENT :
      (awb_mode_ == "daylight" ? Argus::AWB_MODE_DAYLIGHT : Argus::AWB_MODE_AUTO);
    check_argus(controls->setAwbMode(awb), "white balance mode");
    check_argus(controls->setExposureCompensation(exposure_ev_), "exposure compensation");
    RCLCPP_INFO(get_logger(), "Color profile: AWB=%s EV=%.2f RGB gains=[%.8f,%.8f,%.8f]",
      awb_mode_.c_str(), exposure_ev_, gain_r_, gain_g_, gain_b_);
    auto * source = require_interface<Argus::ISourceSettings>(resources.request.get(),
        "ISourceSettings");
    check_argus(source->setSensorMode(modes[sensor_mode_]), "setSensorMode");
    check_argus(source->setFrameDurationRange(Argus::Range<uint64_t>(frame_ns)), "frame duration");
    if (exposure_us_ > 0) {
      const uint64_t exposure_ns = static_cast<uint64_t>(exposure_us_) * 1000;
      const auto range = mode->getExposureTimeRange();
      if (exposure_ns < range.min() || exposure_ns > range.max() || exposure_ns > frame_ns) {
        throw std::invalid_argument("exposure_us is outside sensor/frame duration limits");
      }
      check_argus(source->setExposureTimeRange(
          Argus::Range<uint64_t>(exposure_ns, exposure_ns)), "fixed exposure");
    } else if (max_exposure_us_ > 0) {
      auto exposure = mode->getExposureTimeRange();
      const uint64_t upper = std::min({static_cast<uint64_t>(max_exposure_us_) * 1000,
            exposure.max(), frame_ns});
      if (upper < exposure.min()) {
        throw std::invalid_argument("max_exposure_us is below sensor minimum");
      }
      check_argus(source->setExposureTimeRange(
          Argus::Range<uint64_t>(exposure.min(), upper)), "exposure range");
    }
    check_argus(resources.capture->repeat(resources.request.get()), "start repeat capture");
    resources.repeating = true;
    const auto start = std::chrono::steady_clock::now();
    auto last_report = start;
    uint64_t frames = 0, report_frames = 0;
    FrameWatchdog watchdog(start);
    std::deque<std::shared_ptr<sensor_msgs::msg::Image>> retained;
    std::unique_ptr<NativeEncoder> native;
    auto idle_until = start;
    auto retry_after = start;
    uint64_t next_input = 0;
    int64_t previous_ros_stamp = 0;
    while (!stopping_ && rclcpp::ok(get_node_base_interface()->get_context())) {
      FrameLease lease(resources);
      // R39.2.2 can take over 300 ms to deliver its first sensor frame.
      // After startup keep the short acquisition wait for responsive shutdown.
      const unsigned timeout_us = frames == 0 ? 2000000U : 250000U;
      const auto result = cuEGLStreamConsumerAcquireFrame(
        &resources.connection, &lease.resource, &resources.stream, timeout_us);
      if (result == CUDA_ERROR_LAUNCH_TIMEOUT) {
        watchdog.no_new_frame(std::chrono::steady_clock::now());
        continue;
      }
      check_cu(result, "acquire EGL frame");
      Argus::UniqueObj<EGLStream::MetadataContainer> metadata_container(
        EGLStream::MetadataContainer::create(resources.egl_output->getEGLDisplay(),
        resources.egl_output->getEGLStream()));
      auto * capture_metadata = require_interface<EGLStream::IArgusCaptureMetadata>(
        metadata_container.get(), "IArgusCaptureMetadata");
      auto * metadata = require_interface<Argus::ICaptureMetadata>(
        capture_metadata->getMetadata(), "ICaptureMetadata");
      const uint64_t sensor_stamp = metadata->getSensorTimestamp();
      // EGL mailbox may return the previous frame after a timeout; never republish it.
      if (!watchdog.observe(sensor_stamp, std::chrono::steady_clock::now())) {
        continue;
      }
      // Correlate the acquisition clock with ROS system time, not frame arrival time.
      const auto before = clock_ns(sensor_clock_);
      const auto wall = clock_ns(CLOCK_REALTIME);
      const auto after = clock_ns(sensor_clock_);
      const auto sensor_now = before + (after - before) / 2;
      const auto age_ns = sensor_now - static_cast<int64_t>(sensor_stamp);
      if (age_ns < -100000000LL || age_ns > 5000000000LL) {
        throw std::runtime_error("Sensor clock mismatch (age ns=" + std::to_string(age_ns) +
                "); verify timestamp_clock on this BSP");
      }
      const auto ros_stamp = wall - age_ns;
      if (ros_stamp <= previous_ros_stamp) {
        throw std::runtime_error(
            "System time jumped backwards; restart camera after clock settles");
      }
      previous_ros_stamp = ros_stamp;
      CUeglFrame frame{};
      check_cu(cuGraphicsResourceGetMappedEglFrame(&frame, lease.resource, 0, 0), "map EGL frame");
      if (frame.width != static_cast<unsigned>(width_) ||
        frame.height != static_cast<unsigned>(height_))
      {
        throw std::runtime_error("Argus frame dimensions differ from requested output");
      }
      auto layout = describe_frame(frame, resources);
      layout.gain_r = static_cast<float>(gain_r_);
      layout.gain_g = static_cast<float>(gain_g_);
      layout.gain_b = static_cast<float>(gain_b_);
      layout.rotate_180 = rotation_ == 180;
      auto info = camera_info_->getCameraInfo();
      if (info.k[0] != 0.0 && (info.width != frame.width || info.height != frame.height)) {
        throw std::runtime_error(
            "Calibration dimensions do not match capture; recalibrate this mode");
      }
      info.width = frame.width;
      info.height = frame.height;
      info.header.frame_id = frame_id_;
      info.header.stamp = rclcpp::Time(ros_stamp, RCL_SYSTEM_TIME);
      auto image = std::make_shared<sensor_msgs::msg::Image>();
      image->header = info.header;
      image->width = frame.width;
      image->height = frame.height;
      image->encoding = "rgb8";
      image->step = frame.width * 3;
      image->data = cuda_buffer_backend::allocate_buffer(
        static_cast<size_t>(image->step) * image->height);
      {
        auto write = cuda_buffer_backend::from_output_buffer(image->data, resources.stream);
        convert_rgb(layout, write.get_ptr(), frame.width, frame.height, image->step,
            resources.stream);
        check_cuda(cudaGetLastError(), "YUV to RGB kernel");
      }  // Record readiness before publishing; no CPU wait, no pixel download.
      if (frames == 0) {
        RCLCPP_INFO(get_logger(),
          "Argus -> CUDA arrays -> RGB8 buffer: %ux%u, EGL format=%d planes=%u "
          "backend=%s sensor_age=%.3f ms exposure=%.3f ms rotation=%d",
          frame.width, frame.height, frame.eglColorFormat, frame.planeCount,
          image->data.get_backend_type().c_str(), age_ns / 1e6,
          metadata->getSensorExposureTime() / 1e6, rotation_);
      }
      image_pub_->publish(*image);
      info_pub_->publish(info);
      retained.push_back(image);
      if (retained.size() > retention_frames_) {retained.pop_front();}

      if (video_pub_) {
        const auto now = std::chrono::steady_clock::now();
        const bool demand = video_pub_->get_subscription_count() ||
          (compressed_pub_ && compressed_pub_->get_subscription_count());
        if (!demand) {
          next_input = 0;
          if (native && now >= idle_until) {
            native.reset();
            RCLCPP_INFO(get_logger(), "Released idle native NVENC session");
          }
        } else if (sensor_stamp >= next_input && now >= retry_after) {
          idle_until = now + std::chrono::seconds(2);
          // Advance a sensor-clock deadline to avoid halving fps due to clock jitter.
          const uint64_t period = 1000000000ULL / stream_fps_;
          next_input = next_input && sensor_stamp - next_input < period ?
            next_input + period : sensor_stamp + period;
          if (encoder_input_) {
            // Share immutable RGB storage and its recorded producer event. The view owns a
            // reference to the same original message until the official encoder releases it.
            auto branch = std::make_unique<sensor_msgs::msg::Image>();
            branch->header = image->header;
            branch->width = image->width;
            branch->height = image->height;
            branch->step = image->step;
            branch->encoding = image->encoding;
            auto & src = cuda_buffer_backend::detail::cuda_impl_of(image->data)->get_cuda_buffer();
            cuda_buffer_backend::CudaBuffer view(src.get_device_ptr(), src.size(), src.get_device_id(),
              [image](uint8_t *) {});
            view.set_write_event(src.get_write_event(), false);
            branch->data = rosidl::Buffer<uint8_t>(std::make_unique<cuda_buffer_backend::CudaBufferImpl<uint8_t>>(
                std::move(view), src.size()));
            encoder_input_->publish(std::move(branch));
          } else {
            try {
              if (!native) {
                native = std::make_unique<NativeEncoder>(stream_width_,
                  (stream_width_ * height_ / width_) & ~1, stream_fps_, bitrate_, interval_,
                  [this](int64_t stamp, const uint8_t * data, size_t length) {
                    Video msg;
                    msg.timestamp = rclcpp::Time(stamp, RCL_SYSTEM_TIME);
                    msg.frame_id = frame_id_;
                    msg.format = "h264";
                    msg.data.assign(data, data + length);
                    publish_encoded_packet(*video_pub_, msg);
                  });
                RCLCPP_INFO(get_logger(), "Opened native NVENC session");
              }
              native->submit(layout, width_, height_, ros_stamp, resources.stream);
            } catch (const std::exception & e) {
              native.reset();
              retry_after = now + std::chrono::seconds(2);
              RCLCPP_ERROR(get_logger(), "Video stream failed: %s", e.what());
            }
          }
        }
      }
      ++frames;
      const auto now = std::chrono::steady_clock::now();
      const auto seconds = std::chrono::duration<double>(now - last_report).count();
      if (seconds >= 5.0) {
        RCLCPP_INFO(get_logger(),
          "Published %lu frames, %.2f fps, sensor age %.2f ms, exposure %.3f ms",
          frames, (frames - report_frames) / seconds, age_ns / 1e6,
          metadata->getSensorExposureTime() / 1e6);
        last_report = now;
        report_frames = frames;
      }
      if (max_frames_ > 0 && frames >= static_cast<uint64_t>(max_frames_)) {
        break;
      }
    }
    RCLCPP_INFO(get_logger(), "Capture stopped cleanly after %lu frames", frames);
  }

  int sensor_id_{}, sensor_mode_{}, width_{}, height_{}, max_frames_{}, max_exposure_us_{};
  int rotation_{}, exposure_us_{};
  double fps_{};
  double exposure_ev_{}, gain_r_{}, gain_g_{}, gain_b_{};
  std::string awb_mode_;
  using Video = foxglove_msgs::msg::CompressedVideo;
  using Compressed = sensor_msgs::msg::CompressedImage;
  std::string stream_mode_;
  int stream_width_{}, stream_fps_{}, bitrate_{}, interval_{};
  rclcpp::Publisher<Video>::SharedPtr video_pub_;
  rclcpp::Publisher<Compressed>::SharedPtr compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr encoder_input_;
  rclcpp::Subscription<Compressed>::SharedPtr encoded_sub_;
  size_t retention_frames_{};
  std::string frame_id_;
  clockid_t sensor_clock_{};
  std::atomic<bool> stopping_{false}, failed_{false}, done_{false};
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
  std::thread worker_;
};

std::shared_ptr<rclcpp::Node> make_camera(const rclcpp::NodeOptions & options)
{
  return std::make_shared<Imx219Node>(options);
}

bool capture_failed(const std::shared_ptr<rclcpp::Node> & node)
{
  const auto camera = std::dynamic_pointer_cast<Imx219Node>(node);
  return !camera || camera->failed();
}

}  // namespace imx219

RCLCPP_COMPONENTS_REGISTER_NODE(imx219::Imx219Node)
