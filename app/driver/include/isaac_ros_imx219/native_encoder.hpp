// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <functional>
#include <memory>

#include "isaac_ros_imx219/convert.hpp"

namespace imx219
{
class NativeEncoder
{
public:
  using Output = std::function<void(int64_t, const uint8_t *, size_t)>;
  NativeEncoder(unsigned width, unsigned height, unsigned fps, unsigned bitrate_kbps,
    unsigned interval, Output publish);
  ~NativeEncoder();
  void submit(const Yuv420 & frame, unsigned width, unsigned height,
    int64_t stamp, cudaStream_t stream);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace imx219
