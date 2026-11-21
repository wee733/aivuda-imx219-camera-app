// Run `ipc_smoke sub` and `ipc_smoke pub` as separate processes in one private DDS domain.
// No camera or flight stack required. The subscriber only succeeds for an imported CUDA frame.
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <stdexcept>
#include <string_view>
#include <thread>

#include <cuda_runtime.h>
#include "cuda_buffer/cuda_buffer_api.hpp"
#include "cuda_buffer/cuda_error.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using Image = sensor_msgs::msg::Image;
using namespace std::chrono_literals;

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  const std::string_view mode(argv[1]);
  if (mode != "pub" && mode != "sub") return 2;
  rclcpp::init(argc, argv);
  cudaStream_t stream;
  CUDA_CHECK(cudaStreamCreate(&stream));
  const bool publish = mode == "pub";
  auto node = std::make_shared<rclcpp::Node>(publish ? "smoke_cuda_source" : "smoke_cuda_sink");
  const char topic[] = "imx219_smoke_image";
  constexpr unsigned width = 16, height = 8, size = width * height * 3;
  bool ok = false;
  if (publish) {
    auto publisher = node->create_publisher<Image>(topic, 3);
    std::deque<Image::UniquePtr> retained;
    bool matched = false;
    for (int seq = 1; seq <= 60 && rclcpp::ok(); ++seq) {
      if (publisher->get_subscription_count() > 0) {
        matched = true;
      } else if (matched) {
        break;  // The smoke subscriber exits after verifying its first imported frame.
      }
      auto msg = std::make_unique<Image>();
      msg->height = height; msg->width = width; msg->step = width * 3; msg->encoding = "rgb8";
      msg->header.stamp.sec = seq;
      msg->data = cuda_buffer_backend::allocate_buffer(size);
      {
        auto write = cuda_buffer_backend::from_output_buffer(msg->data, stream);
        CUDA_CHECK(cudaMemsetAsync(write.get_ptr(), seq, size, stream));
      }
      publisher->publish(*msg);
      retained.push_back(std::move(msg));
      if (retained.size() > 12) retained.pop_front();
      std::this_thread::sleep_for(100ms);
    }
    ok = true;
  } else {
    rclcpp::SubscriptionOptions options;
    options.acceptable_buffer_backends = "any";
    auto sub = node->create_subscription<Image>(topic, 3, [&](Image::ConstSharedPtr msg) {
      if (msg->data.get_backend_type() != "cuda" || msg->data.size() != size) return;
      auto read = cuda_buffer_backend::from_input_buffer(msg->data, stream);
      std::array<uint8_t, size> pixels;
      CUDA_CHECK(cudaMemcpyAsync(pixels.data(), read.get_ptr(), size, cudaMemcpyDeviceToHost, stream));
      CUDA_CHECK(cudaStreamSynchronize(stream));
      if (!std::all_of(pixels.begin(), pixels.end(), [&](uint8_t v) {return v == msg->header.stamp.sec;}))
        throw std::runtime_error("CUDA data differs from the publishing frame");
      std::printf("Verified CUDA IPC frame %d: %u bytes\n", msg->header.stamp.sec, size);
      ok = true;
    }, options);
    const auto deadline = std::chrono::steady_clock::now() + 12s;
    while (rclcpp::ok() && !ok && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(5ms);
    }
  }
  node.reset();
  CUDA_CHECK(cudaStreamDestroy(stream));
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
