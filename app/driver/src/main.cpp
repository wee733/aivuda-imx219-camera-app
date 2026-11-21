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

#include <exception>
#include "isaac_ros_imx219/camera_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = imx219::make_camera(rclcpp::NodeOptions());
    rclcpp::spin(node);
    result = imx219::capture_failed(node) ? 1 : 0;
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("imx219"), "%s", error.what());
    result = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
