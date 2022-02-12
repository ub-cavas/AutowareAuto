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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "dataspeed_ford_interface/dataspeed_ford_interface_node.hpp"
#include "dataspeed_ford_interface/dataspeed_ford_interface.hpp"

#include <common/types.hpp>

#include <memory>
#include <string>
#include <unordered_set>

namespace autoware
{
namespace dataspeed_ford_interface
{
using autoware::common::types::float32_t;
using autoware::drivers::vehicle_interface::ViFeature;

DataspeedFordInterfaceNode::DataspeedFordInterfaceNode(const rclcpp::NodeOptions & options)
: VehicleInterfaceNode {
    "dataspeed_ford_interface",
    std::unordered_set<ViFeature> {
        ViFeature::HEADLIGHTS,
        ViFeature::HORN,
        ViFeature::WIPERS,
        ViFeature::GEAR,
      },
      options
}
{
  set_interface(
    std::make_unique<DataspeedFordInterface>(
      *this,
      declare_parameter("dataspeed_ford.ecu_build_num").get<uint16_t>(),
      declare_parameter("dataspeed_ford.front_axle_to_cog").get<float32_t>(),
      declare_parameter("dataspeed_ford.rear_axle_to_cog").get<float32_t>(),
      declare_parameter("dataspeed_ford.steer_to_tire_ratio").get<float32_t>(),
      declare_parameter("dataspeed_ford.max_steer_angle").get<float32_t>(),
      get_state_machine().get_config().accel_limits().max(),
      get_state_machine().get_config().accel_limits().min(),
      declare_parameter("dataspeed_ford.acceleration_positive_jerk_limit").get<float32_t>(),
      declare_parameter("dataspeed_ford.deceleration_negative_jerk_limit").get<float32_t>(),
      declare_parameter("dataspeed_ford.pub_period").get<uint32_t>()
  ));
}
}  // namespace dataspeed_ford_interface
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::dataspeed_ford_interface::DataspeedFordInterfaceNode)
