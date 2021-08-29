// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the InteractiveTrajectorySpoofer class.

#ifndef INTERACTIVE_TRAJECTORY_SPOOFER__INTERACTIVE_TRAJECTORY_SPOOFER_HPP_
#define INTERACTIVE_TRAJECTORY_SPOOFER__INTERACTIVE_TRAJECTORY_SPOOFER_HPP_

#include <interactive_trajectory_spoofer/visibility_control.hpp>

#include <autoware_auto_msgs/msg/complex32.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <common/types.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>

namespace autoware
{
namespace interactive_trajectory_spoofer
{
using Complex32 = autoware_auto_msgs::msg::Complex32;
using Trajectory = autoware_auto_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_msgs::msg::TrajectoryPoint;

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;


struct INTERACTIVE_TRAJECTORY_SPOOFER_PUBLIC ControlPoint
{
  float64_t x;
  float64_t y;
};

class INTERACTIVE_TRAJECTORY_SPOOFER_PUBLIC InteractiveTrajectorySpoofer
{
};
}  // namespace interactive_trajectory_spoofer
}  // namespace autoware

#endif  // INTERACTIVE_TRAJECTORY_SPOOFER__INTERACTIVE_TRAJECTORY_SPOOFER_HPP_
