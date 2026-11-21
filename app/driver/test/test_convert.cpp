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

#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "isaac_ros_imx219/convert.hpp"

void verify_conversion(bool full_range, bool swap_uv, bool rotate_180 = false)
{
  // Four 2x2 blocks: black, white, red, blue; BT.601 limited-range test vectors.
  std::array<uint8_t, 16> y = {16, 16, 235, 235, 16, 16, 235, 235,
    81, 81, 41, 41, 81, 81, 41, 41};
  std::array<uint8_t, 4> u = {128, 128, 90, 240};
  std::array<uint8_t, 4> v = {128, 128, 240, 110};
  if (full_range) {
    y = {0, 0, 255, 255, 0, 0, 255, 255, 76, 76, 29, 29, 76, 76, 29, 29};
    u = {128, 128, 85, 255};
    v = {128, 128, 255, 107};
  }
  if (swap_uv) {
    std::swap(u, v);
  }
  const std::array<uint8_t, 8> uv = {u[0], v[0], u[1], v[1], u[2], v[2], u[3], v[3]};
  const std::array<std::array<int, 3>, 4> expected = {{{0, 0, 0}, {255, 255, 255},
    {255, 0, 0}, {0, 0, 255}}};
  for (bool semiplanar : {false, true}) {
    cudaStream_t stream{};
    ASSERT_EQ(cudaStreamCreate(&stream), cudaSuccess);
    std::array<cudaArray_t, 3> arrays{};
    std::array<cudaSurfaceObject_t, 3> surfaces{};
    for (unsigned plane = 0; plane < (semiplanar ? 2U : 3U); ++plane) {
      const unsigned width = plane == 0 ? 4 : 2;
      const unsigned height = plane == 0 ? 4 : 2;
      const auto channels = plane == 1 && semiplanar ?
        cudaCreateChannelDesc<uchar2>() : cudaCreateChannelDesc<unsigned char>();
      ASSERT_EQ(cudaMallocArray(&arrays[plane], &channels, width, height,
        cudaArraySurfaceLoadStore), cudaSuccess);
      const unsigned row_bytes = width * ((plane == 1 && semiplanar) ? 2 : 1);
      const auto *source = plane == 0 ? y.data() :
        (semiplanar ? uv.data() : (plane == 1 ? u.data() : v.data()));
      ASSERT_EQ(cudaMemcpy2DToArray(arrays[plane], 0, 0, source, row_bytes,
        row_bytes, height, cudaMemcpyHostToDevice), cudaSuccess);
      cudaResourceDesc descriptor{};
      descriptor.resType = cudaResourceTypeArray;
      descriptor.res.array.array = arrays[plane];
      ASSERT_EQ(cudaCreateSurfaceObject(&surfaces[plane], &descriptor), cudaSuccess);
    }
    uint8_t *output{};
    constexpr unsigned step = 19;
    ASSERT_EQ(cudaMalloc(&output, step * 4), cudaSuccess);
    ASSERT_EQ(cudaMemsetAsync(output, 0xA5, step * 4, stream), cudaSuccess);
    imx219::Yuv420 input;
    input.y = surfaces[0];
    input.u = surfaces[1];
    input.v = surfaces[2];
    input.full_range = full_range;
    input.swap_uv = swap_uv;
    input.semiplanar = semiplanar;
    input.rotate_180 = rotate_180;
    imx219::convert_rgb(input, output, 4, 4, step, stream);
    ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
    std::vector<uint8_t> result(step * 4);
    ASSERT_EQ(cudaMemcpy(result.data(), output, result.size(), cudaMemcpyDeviceToHost),
      cudaSuccess);
    for (unsigned row = 0; row < 4; ++row) {
      for (unsigned col = 0; col < 4; ++col) {
        const unsigned block = (row / 2) * 2 + col / 2;
        const auto & reference = expected[rotate_180 ? 3 - block : block];
        for (unsigned channel = 0; channel < 3; ++channel) {
          EXPECT_NEAR(result[row * step + col * 3 + channel], reference[channel], 2);
        }
      }
      for (unsigned byte = 12; byte < step; ++byte) {
        EXPECT_EQ(result[row * step + byte], 0xA5);
      }
    }
    cudaFree(output);
    for (unsigned plane = 0; plane < (semiplanar ? 2U : 3U); ++plane) {
      cudaDestroySurfaceObject(surfaces[plane]);
      cudaFreeArray(arrays[plane]);
    }
    cudaStreamDestroy(stream);
  }
}

TEST(Conversion, LimitedRangePlanarAndSemiplanar)
{
  verify_conversion(false, false);
  verify_conversion(false, true);
}

TEST(Conversion, FullRangePlanarAndSemiplanar)
{
  verify_conversion(true, false);
  verify_conversion(true, true);
}

TEST(Conversion, Rotate180PreservesColorsAndRowPadding)
{
  for (bool full_range : {false, true}) {
    for (bool swap_uv : {false, true}) {
      verify_conversion(full_range, swap_uv, true);
    }
  }
}


TEST(Conversion, GuideLutAllByteValues)
{
  constexpr unsigned width = 256, height = 2;
  std::array<uint8_t, width * height> y{};
  for (unsigned i = 0; i < y.size(); ++i) {
    y[i] = static_cast<uint8_t>(i % width);
  }
  std::array<uint8_t, width> uv{};
  uv.fill(128);
  cudaArray_t y_array{}, uv_array{};
  auto y_channel = cudaCreateChannelDesc<unsigned char>();
  auto uv_channel = cudaCreateChannelDesc<uchar2>();
  ASSERT_EQ(cudaMallocArray(&y_array, &y_channel, width, height,
      cudaArraySurfaceLoadStore), cudaSuccess);
  ASSERT_EQ(cudaMallocArray(&uv_array, &uv_channel, width / 2, height / 2,
      cudaArraySurfaceLoadStore), cudaSuccess);
  ASSERT_EQ(cudaMemcpy2DToArray(y_array, 0, 0, y.data(), width, width, height,
      cudaMemcpyHostToDevice), cudaSuccess);
  ASSERT_EQ(cudaMemcpy2DToArray(uv_array, 0, 0, uv.data(), width, width, height / 2,
      cudaMemcpyHostToDevice), cudaSuccess);
  imx219::Yuv420 input;
  cudaResourceDesc descriptor{};
  descriptor.resType = cudaResourceTypeArray;
  descriptor.res.array.array = y_array;
  ASSERT_EQ(cudaCreateSurfaceObject(&input.y, &descriptor), cudaSuccess);
  descriptor.res.array.array = uv_array;
  ASSERT_EQ(cudaCreateSurfaceObject(&input.u, &descriptor), cudaSuccess);
  input.full_range = true;
  input.semiplanar = true;
  // The guide stores BGR. The published image and this kernel use RGB.
  input.gain_r = 1.1772151898734178f;
  input.gain_g = 1.f;
  input.gain_b = 0.8691588785046729f;
  const std::array<float, 3> gains = {input.gain_r, input.gain_g, input.gain_b};
  uint8_t * output{};
  constexpr unsigned bytes = width * height * 3;
  ASSERT_EQ(cudaMalloc(&output, bytes), cudaSuccess);
  imx219::convert_rgb(input, output, width, height, width * 3, nullptr);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  std::array<uint8_t, bytes> actual{};
  ASSERT_EQ(cudaMemcpy(actual.data(), output, bytes, cudaMemcpyDeviceToHost), cudaSuccess);
  for (unsigned i = 0; i < y.size(); ++i) {
    for (unsigned c = 0; c < 3; ++c) {
      const auto expected = static_cast<uint8_t>(
        std::nearbyint(std::min(255.f, static_cast<float>(y[i]) * gains[c])));
      EXPECT_EQ(actual[i * 3 + c], expected) << "value=" << unsigned(y[i]) << " channel=" << c;
    }
  }
  cudaFree(output);
  cudaDestroySurfaceObject(input.y);
  cudaDestroySurfaceObject(input.u);
  cudaFreeArray(y_array);
  cudaFreeArray(uv_array);
}
