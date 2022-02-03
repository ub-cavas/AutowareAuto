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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the vesc_interface_node class.

#ifndef VESC_INTERFACE__VESC_INTERFACE_NODE_HPP_
#define VESC_INTERFACE__VESC_INTERFACE_NODE_HPP_

#include <vesc_interface/vesc_interface.hpp>

#include <vehicle_interface/vehicle_interface_node.hpp>

#include <rclcpp/rclcpp.hpp>

namespace autoware
{
namespace vesc_interface
{
/// \class VESCInterfaceNode
/// \brief ROS 2 Node for hello world.
class VESC_INTERFACE_PUBLIC VESCInterfaceNode
  : public ::autoware::drivers::vehicle_interface::VehicleInterfaceNode
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit VESCInterfaceNode(const rclcpp::NodeOptions & options);
};
}  // namespace vesc_interface
}  // namespace autoware

#endif  // VESC_INTERFACE__VESC_INTERFACE_NODE_HPP_
