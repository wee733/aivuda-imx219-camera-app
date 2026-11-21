// SPDX-License-Identifier: Apache-2.0
#ifndef ISAAC_ROS_IMX219__FRAME_WATCHDOG_HPP_
#define ISAAC_ROS_IMX219__FRAME_WATCHDOG_HPP_

#include <chrono>
#include <cstdint>
#include <stdexcept>

namespace imx219
{
class FrameWatchdog
{
public:
  using Clock = std::chrono::steady_clock;
  explicit FrameWatchdog(Clock::time_point start) : last_new_frame_(start) {}

  void no_new_frame(Clock::time_point now) const
  {
    if (now - last_new_frame_ >= std::chrono::seconds(10)) {
      throw std::runtime_error("No new CSI sensor frame for 10 seconds");
    }
  }

  bool observe(uint64_t stamp, Clock::time_point now)
  {
    if (stamp == last_stamp_) {
      no_new_frame(now);
      return false;
    }
    if (stamp < last_stamp_) {
      throw std::runtime_error("Non-monotonic sensor timestamp");
    }
    last_stamp_ = stamp;
    last_new_frame_ = now;
    return true;
  }

private:
  uint64_t last_stamp_{};
  Clock::time_point last_new_frame_;
};
}  // namespace imx219

#endif  // ISAAC_ROS_IMX219__FRAME_WATCHDOG_HPP_
