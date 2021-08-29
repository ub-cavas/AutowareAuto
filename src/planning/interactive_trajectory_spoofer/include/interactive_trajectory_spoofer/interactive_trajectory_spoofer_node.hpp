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
/// \brief This file defines the InteractiveTrajectorySpooferNode class.

#ifndef INTERACTIVE_TRAJECTORY_SPOOFER__INTERACTIVE_TRAJECTORY_SPOOFER_NODE_HPP_
#define INTERACTIVE_TRAJECTORY_SPOOFER__INTERACTIVE_TRAJECTORY_SPOOFER_NODE_HPP_

#include <interactive_trajectory_spoofer/interactive_trajectory_spoofer.hpp>
#include <interactive_trajectory_spoofer/visibility_control.hpp>

#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <common/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace autoware
{
namespace interactive_trajectory_spoofer
{
using Trajectory = autoware_auto_msgs::msg::Trajectory;
using std::placeholders::_1;

/// \class InteractiveTrajectorySpooferNode
/// \brief ROS 2 Node for creating interactive fake trajectories
class INTERACTIVE_TRAJECTORY_SPOOFER_PUBLIC InteractiveTrajectorySpooferNode
  : public rclcpp::Node
{
public:
  explicit InteractiveTrajectorySpooferNode(const rclcpp::NodeOptions & node_options);

private:
};

}  // namespace interactive_trajectory_spoofer
}  // namespace autoware

#endif  // INTERACTIVE_TRAJECTORY_SPOOFER__INTERACTIVE_TRAJECTORY_SPOOFER_NODE_HPP_
