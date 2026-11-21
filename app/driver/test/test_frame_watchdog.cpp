// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "isaac_ros_imx219/frame_watchdog.hpp"

using namespace std::chrono_literals;
using imx219::FrameWatchdog;

TEST(FrameWatchdog, StartupWithoutFramesOrWithZeroTimestampsTimesOut)
{
  FrameWatchdog::Clock::time_point start{};
  FrameWatchdog watchdog(start);
  EXPECT_FALSE(watchdog.observe(0, start + 9s));
  EXPECT_THROW(watchdog.observe(0, start + 10s), std::runtime_error);
  EXPECT_THROW(watchdog.no_new_frame(start + 10s), std::runtime_error);
}

TEST(FrameWatchdog, OnlyNewTimestampsExtendDeadline)
{
  FrameWatchdog::Clock::time_point start{};
  FrameWatchdog watchdog(start);
  ASSERT_TRUE(watchdog.observe(100, start + 2s));
  EXPECT_FALSE(watchdog.observe(100, start + 9s));
  EXPECT_NO_THROW(watchdog.no_new_frame(start + 11s));
  ASSERT_TRUE(watchdog.observe(101, start + 11s));
  EXPECT_FALSE(watchdog.observe(101, start + 20s));
  EXPECT_THROW(watchdog.no_new_frame(start + 21s), std::runtime_error);
  EXPECT_THROW(watchdog.observe(101, start + 21s), std::runtime_error);
  EXPECT_THROW(watchdog.observe(99, start + 21s), std::runtime_error);
}
