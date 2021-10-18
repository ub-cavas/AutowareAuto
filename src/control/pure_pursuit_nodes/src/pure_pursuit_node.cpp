// Copyright 2019 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <common/types.hpp>
#include <motion_common/motion_common.hpp>

#include <memory>
#include <string>

#include "pure_pursuit_nodes/pure_pursuit_node.hpp"

using autoware::common::types::float64_t;
using autoware::common::types::float32_t;
using autoware::common::types::bool8_t;

namespace autoware
{
namespace motion
{
namespace control
{
/// \brief Resources relating to the pure pursuit node package
namespace pure_pursuit_nodes
{
PurePursuitNode::PurePursuitNode(
  const std::string & node_name,
  const std::string & node_namespace)
: ControllerBaseNode{node_name, node_namespace, "ctrl_cmd", "current_pose",
    "tf", "trajectory", "ctrl_diag"}
{
  pure_pursuit::Config cfg{
    static_cast<float32_t>(declare_parameter<float64_t>(
      "controller.minimum_lookahead_distance")),
    static_cast<float32_t>(declare_parameter<float64_t>(
      "controller.maximum_lookahead_distance")),
    static_cast<float32_t>(
      declare_parameter<float64_t>("controller.speed_to_lookahead_ratio")),
    declare_parameter<bool8_t>("controller.is_interpolate_lookahead_point"),
    declare_parameter<bool8_t>("controller.is_delay_compensation"),
    static_cast<float32_t>(declare_parameter<float64_t>(
      "controller.emergency_stop_distance")),
    static_cast<float32_t>(declare_parameter<float64_t>(
      "controller.speed_thres_traveling_direction")),
    static_cast<float32_t>(declare_parameter<float64_t>(
      "controller.dist_front_rear_wheels"))};
  set_controller(std::make_unique<pure_pursuit::PurePursuit>(cfg));
}
////////////////////////////////////////////////////////////////////////////////
PurePursuitNode::PurePursuitNode(
  const std::string & node_name,
  const pure_pursuit::Config & cfg,
  const std::string & node_namespace)
: ControllerBaseNode{node_name, node_namespace, "ctrl_cmd", "current_pose",
    "tf", "trajectory", "ctrl_diag"}
{
  set_controller(std::make_unique<pure_pursuit::PurePursuit>(cfg));
}
}  // namespace pure_pursuit_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware
