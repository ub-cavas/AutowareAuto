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

#include "interactive_trajectory_spoofer/interactive_trajectory_spoofer_node.hpp"
#include "interactive_trajectory_spoofer/interactive_trajectory_spoofer.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <memory>
#include <string>


namespace autoware
{
namespace interactive_trajectory_spoofer
{
InteractiveTrajectorySpooferNode::InteractiveTrajectorySpooferNode(
  const rclcpp::NodeOptions & node_options)
: Node{"interactive_trajectory_spoofer_node", node_options}
{}
}  // namespace interactive_trajectory_spoofer
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::interactive_trajectory_spoofer::InteractiveTrajectorySpooferNode)
