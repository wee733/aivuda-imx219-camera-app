// SPDX-License-Identifier: Apache-2.0
#include "isaac_ros_imx219/native_encoder.hpp"

#include <NvVideoEncoder.h>
#include <nvbufsurface.h>

#include <array>
#include <cerrno>
#include <deque>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace imx219
{
namespace
{
void checked(int status, const char * operation)
{
  if (status != 0) {
    throw std::runtime_error(std::string("NVENC ") + operation + " failed (" +
        std::to_string(status) + ", errno " + std::to_string(errno) + ")");
  }
}

// Each output plane is a pitch-linear NV12 DMA-BUF. CUDA writes into the same
// allocation that the encoder imports; only the compressed access unit is read on the CPU.
struct InputSurface
{
  NvBufSurface * surface{};
  CUgraphicsResource resource{};
  CUeglFrame frame{};
  bool mapped{};

  ~InputSurface()
  {
    if (resource) {cuGraphicsUnregisterResource(resource);}
    if (mapped) {NvBufSurfaceUnMapEglImage(surface, 0);}
    if (surface) {NvBufSurfaceDestroy(surface);}
  }

  void allocate(unsigned width, unsigned height, NvBufSurfaceColorFormat format)
  {
    NvBufSurfaceAllocateParams args{};
    args.params.width = width;
    args.params.height = height;
    args.params.colorFormat = format;
    args.params.layout = NVBUF_LAYOUT_PITCH;
    args.params.memType = NVBUF_MEM_SURFACE_ARRAY;
    args.memtag = NvBufSurfaceTag_VIDEO_ENC;
    checked(NvBufSurfaceAllocate(&surface, 1, &args), "allocate surface");
    checked(NvBufSurfaceMapEglImage(surface, 0), "map EGL image");
    mapped = true;
    checked(cuGraphicsEGLRegisterImage(&resource,
      surface->surfaceList[0].mappedAddr.eglImage, CU_GRAPHICS_MAP_RESOURCE_FLAGS_WRITE_DISCARD),
      "register CUDA image");
    checked(cuGraphicsResourceGetMappedEglFrame(&frame, resource, 0, 0), "map CUDA frame");
    if (frame.frameType != CU_EGL_FRAME_TYPE_PITCH || frame.planeCount != 2) {
      throw std::runtime_error("Expected two pitch-linear CUDA planes for NVENC");
    }
  }
};

struct QueueBuffer
{
  v4l2_buffer buffer{};
  std::array<v4l2_plane, VIDEO_MAX_PLANES> planes{};
  QueueBuffer() {buffer.m.planes = planes.data(); buffer.length = planes.size();}
};
}  // namespace

struct NativeEncoder::Impl
{
  // Release encoder queues before unmapping the surfaces.
  std::vector<std::unique_ptr<InputSurface>> surfaces;
  std::unique_ptr<NvVideoEncoder> encoder;
  unsigned width, height;
  size_t initial_slot{};
  std::deque<int64_t> stamps;
  Output publish;

  Impl(unsigned w, unsigned h, unsigned fps, unsigned bitrate, unsigned interval, const Yuv420 & input, Output sink)
  : encoder(NvVideoEncoder::createVideoEncoder("imx219-nvenc", O_NONBLOCK)),
    width(w), height(h), publish(std::move(sink))
  {
    if (!encoder) {throw std::runtime_error("Cannot open Jetson NVENC");}
    checked(encoder->setCapturePlaneFormat(V4L2_PIX_FMT_H264, w, h, 2 * 1024 * 1024), "H264 format");
    // write_nv12 preserves Argus sample values; tag the buffer and bitstream accordingly.
    const auto cs = input.matrix == 2 ? V4L2_COLORSPACE_BT2020 :
      (input.matrix == 1 ? V4L2_COLORSPACE_REC709 : V4L2_COLORSPACE_SMPTE170M);
    const auto format = input.matrix == 2 ? NVBUF_COLOR_FORMAT_NV12_2020 :
      (input.matrix == 1 ? (input.full_range ? NVBUF_COLOR_FORMAT_NV12_709_ER : NVBUF_COLOR_FORMAT_NV12_709) :
       (input.full_range ? NVBUF_COLOR_FORMAT_NV12_ER : NVBUF_COLOR_FORMAT_NV12));
    checked(encoder->setOutputPlaneFormat(V4L2_PIX_FMT_NV12M, w, h, cs, input.full_range), "NV12 format");
    checked(encoder->setBitrate(bitrate * 1000), "bitrate");
    checked(encoder->setProfile(V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE), "profile");
    checked(encoder->setRateControlMode(V4L2_MPEG_VIDEO_BITRATE_MODE_CBR), "rate control");
    checked(encoder->setIFrameInterval(interval), "I-frame interval");
    checked(encoder->setIDRInterval(interval), "IDR interval");
    checked(encoder->setFrameRate(fps, 1), "frame rate");
    checked(encoder->setNumBFrames(0), "disable B-frames");
    checked(encoder->setInsertSpsPpsAtIdrEnabled(true), "SPS/PPS");
    checked(encoder->setInsertVuiEnabled(true), "insert color VUI");
    if (input.full_range) {
      checked(encoder->setExtendedColorFormat(true), "full-range video");
    }
    checked(encoder->setHWPresetType(V4L2_ENC_HW_PRESET_ULTRAFAST), "hardware preset");
    checked(encoder->setMaxPerfMode(1), "maximum performance");
    checked(encoder->output_plane.reqbufs(V4L2_MEMORY_DMABUF, 4), "request input buffers");
    checked(encoder->capture_plane.setupPlane(V4L2_MEMORY_MMAP, 4, true, false), "map output buffers");
    for (unsigned i = 0; i < encoder->output_plane.getNumBuffers(); ++i) {
      surfaces.push_back(std::make_unique<InputSurface>());
      surfaces.back()->allocate(w, h, format);
    }
    checked(encoder->output_plane.setStreamStatus(true), "start input");
    checked(encoder->capture_plane.setStreamStatus(true), "start output");
    for (unsigned i = 0; i < encoder->capture_plane.getNumBuffers(); ++i) {
      QueueBuffer slot;
      slot.buffer.index = i;
      checked(encoder->capture_plane.qBuffer(slot.buffer, nullptr), "queue capture buffer");
    }
  }

  void drain()
  {
    while (!stamps.empty()) {
      QueueBuffer slot;
      NvBuffer * output{};
      if (encoder->capture_plane.dqBuffer(slot.buffer, &output, nullptr, 0) < 0) {
        if (errno == EAGAIN) {return;}
        throw std::runtime_error("NVENC failed to dequeue encoded frame");
      }
      if (!output || !output->planes[0].bytesused || (slot.buffer.flags & V4L2_BUF_FLAG_ERROR)) {
        throw std::runtime_error("NVENC returned invalid access unit");
      }
      publish(stamps.front(), output->planes[0].data, output->planes[0].bytesused);
      stamps.pop_front();
      checked(encoder->capture_plane.qBuffer(slot.buffer, nullptr), "requeue capture buffer");
    }
  }

  void submit(const Yuv420 & frame, unsigned w, unsigned h, int64_t stamp, cudaStream_t stream)
  {
    drain();
    QueueBuffer slot;
    if (initial_slot < surfaces.size()) {
      slot.buffer.index = initial_slot++;
    } else if (encoder->output_plane.dqBuffer(slot.buffer, nullptr, nullptr, 0) < 0) {
      if (errno == EAGAIN) {return;}  // Drop before encoding if the hardware is busy.
      throw std::runtime_error("NVENC failed to dequeue input buffer");
    }
    auto & target = *surfaces.at(slot.buffer.index);
    const auto & params = target.surface->surfaceList[0];
    convert_nv12(frame, w, h, width, height,
      static_cast<uint8_t *>(target.frame.frame.pPitch[0]), params.planeParams.pitch[0],
      static_cast<uint8_t *>(target.frame.frame.pPitch[1]), params.planeParams.pitch[1], stream);
    checked(cudaGetLastError(), "NV12 kernel");
    checked(cudaStreamSynchronize(stream), "finish CUDA writes before NVENC reads");
    for (unsigned i = 0; i < 2; ++i) {
      slot.planes[i].m.fd = static_cast<int>(params.bufferDesc);
      slot.planes[i].bytesused = params.planeParams.psize[i];
    }
    checked(encoder->output_plane.qBuffer(slot.buffer, nullptr), "queue NV12 input");
    stamps.push_back(stamp);  // No B-frames, one complete access unit per capture buffer.
    drain();
  }
};

NativeEncoder::NativeEncoder(unsigned w, unsigned h, unsigned fps, unsigned bitrate,
  unsigned interval, const Yuv420 & input, Output sink)
: impl_(std::make_unique<Impl>(w, h, fps, bitrate, interval, input, std::move(sink))) {}
NativeEncoder::~NativeEncoder() = default;
void NativeEncoder::submit(const Yuv420 & frame, unsigned w, unsigned h, int64_t stamp,
  cudaStream_t stream)
{
  impl_->submit(frame, w, h, stamp, stream);
}
}  // namespace imx219
