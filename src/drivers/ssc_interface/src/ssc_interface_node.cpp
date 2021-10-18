// Copyright 2020 The Autoware Foundation
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

#include "ssc_interface/ssc_interface_node.hpp"
#include "ssc_interface/ssc_interface.hpp"

#include <common/types.hpp>

#include <memory>
#include <string>
#include <unordered_set>

namespace ssc_interface
{

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::drivers::vehicle_interface::ViFeature;

SscInterfaceNode::SscInterfaceNode(const rclcpp::NodeOptions & options)
: VehicleInterfaceNode{
    "ssc_interface",
    std::unordered_set<ViFeature> {},
    options
}
{
  set_interface(
    std::make_unique<SscInterface>(
      *this,
      static_cast<float32_t>(declare_parameter<float64_t>("ssc.front_axle_to_cog")),
      static_cast<float32_t>(declare_parameter<float64_t>("ssc.rear_axle_to_cog")),
      get_state_machine().get_config().accel_limits().max(),
      get_state_machine().get_config().accel_limits().min(),
      static_cast<float32_t>(declare_parameter<float64_t>("ssc.max_yaw_rate"))
  ));
}

}  // namespace ssc_interface

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(ssc_interface::SscInterfaceNode)
