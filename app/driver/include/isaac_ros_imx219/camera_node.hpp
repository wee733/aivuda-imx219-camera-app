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

#ifndef ISAAC_ROS_IMX219__CAMERA_NODE_HPP_
#define ISAAC_ROS_IMX219__CAMERA_NODE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"

namespace imx219
{
std::shared_ptr<rclcpp::Node> make_camera(const rclcpp::NodeOptions & options);
bool capture_failed(const std::shared_ptr<rclcpp::Node> & node);
}  // namespace imx219

#endif  // ISAAC_ROS_IMX219__CAMERA_NODE_HPP_
