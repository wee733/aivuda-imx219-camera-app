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

#include "isaac_ros_imx219/convert.hpp"

namespace imx219
{

__device__ uint8_t clip(float x)
{
  return static_cast<uint8_t>(fminf(255.f, fmaxf(0.f, x + .5f)));
}

__device__ uint8_t correct(uint8_t value, float gain)
{
  // Match the guide's float32 LUT: clamp, then NumPy round-to-nearest-even.
  return static_cast<uint8_t>(__float2uint_rn(fminf(255.f, value * gain)));
}

__global__ void yuv_to_rgb(
  Yuv420 in, uint8_t * out, unsigned w, unsigned h, unsigned step)
{
  const unsigned x = blockIdx.x * blockDim.x + threadIdx.x;
  const unsigned y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= w || y >= h) {
    return;
  }
  const unsigned sx = in.rotate_180 ? w - 1 - x : x;
  const unsigned sy = in.rotate_180 ? h - 1 - y : y;
  float yy = surf2Dread<unsigned char>(in.y, sx, sy);
  float u, v;
  if (in.semiplanar) {
    const uchar2 uv = surf2Dread<uchar2>(in.u, (sx / 2) * sizeof(uchar2), sy / 2);
    u = uv.x;
    v = uv.y;
  } else {
    u = surf2Dread<unsigned char>(in.u, sx / 2, sy / 2);
    v = surf2Dread<unsigned char>(in.v, sx / 2, sy / 2);
  }
  if (in.swap_uv) {
    const float t = u;
    u = v;
    v = t;
  }
  u -= 128.f;
  v -= 128.f;
  if (!in.full_range) {
    yy = (yy - 16.f) * (255.f / 219.f);
    u *= 255.f / 224.f;
    v *= 255.f / 224.f;
  }
  float kr = .299f, kb = .114f;
  if (in.matrix == 1) {
    kr = .2126f;
    kb = .0722f;
  }
  if (in.matrix == 2) {
    kr = .2627f;
    kb = .0593f;
  }
  const float kg = 1.f - kr - kb;
  uint8_t * p = out + y * step + x * 3;
  p[0] = correct(clip(yy + 2.f * (1.f - kr) * v), in.gain_r);
  p[1] = correct(
    clip(yy - 2.f * kb * (1.f - kb) / kg * u - 2.f * kr * (1.f - kr) / kg * v), in.gain_g);
  p[2] = correct(clip(yy + 2.f * (1.f - kb) * u), in.gain_b);
}

void convert_rgb(
  const Yuv420 & in, uint8_t * out, unsigned w, unsigned h, unsigned step, cudaStream_t stream)
{
  yuv_to_rgb<<<dim3((w + 31) / 32, (h + 7) / 8), dim3(32, 8), 0, stream>>>(in, out, w, h, step);
}

}  // namespace imx219
