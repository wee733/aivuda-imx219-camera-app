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

#ifndef ISAAC_ROS_IMX219__CONVERT_HPP_
#define ISAAC_ROS_IMX219__CONVERT_HPP_
#include <cuda.h>
#include <cudaEGL.h>
#include <cuda_runtime.h>
#include <cstdint>

namespace imx219
{
// Surfaces remain valid until all kernels using them have finished.
struct Yuv420
{
  cudaSurfaceObject_t y{}, u{}, v{};
  bool semiplanar{}, swap_uv{}, full_range{};
  bool rotate_180{};  // Rotate both axes without reflection or interpolation.
  int matrix{};  // 0=BT.601, 1=BT.709, 2=BT.2020
  float gain_r{1.f}, gain_g{1.f}, gain_b{1.f};  // Post-ISP 8-bit RGB LUT gains.
};
void convert_rgb(
  const Yuv420 & input, uint8_t * output, unsigned width,
  unsigned height, unsigned step, cudaStream_t stream);
}  // namespace imx219

#endif  // ISAAC_ROS_IMX219__CONVERT_HPP_
