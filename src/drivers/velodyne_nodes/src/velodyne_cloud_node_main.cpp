// Copyright 2018-2020 the Autoware Foundation, Arm Limited
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

#include <velodyne_nodes/velodyne_cloud_node.hpp>
#include <rcutils/cmdline_parser.h>

//lint -e537 NOLINT  // cpplint vs pclint
#include <string>
//lint -e537 NOLINT  // cpplint vs pclint
#include <memory>
#include <vector>
#include <cstdio>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace autoware
{
namespace drivers
{
namespace velodyne_nodes
{

class VelodyneCloudWrapperNode : public rclcpp::Node
{
private:
  autoware::drivers::velodyne_nodes::VLP16DriverNode::SharedPtr vlp16_driver_node_ = nullptr;
  autoware::drivers::velodyne_nodes::VLP32CDriverNode::SharedPtr vlp32_c_driver_node_ = nullptr;
  autoware::drivers::velodyne_nodes::VLS128DriverNode::SharedPtr vls128_driver_node_ = nullptr;

public:
  explicit VelodyneCloudWrapperNode(const rclcpp::NodeOptions & node_options)
  : Node("velodyne_cloud_wrapper_node", node_options)
  {
    // std::string model = this->declare_parameter("model").template get<std::string>();
    std::string model = "vlp16";

    if (model == "vlp16") {
      vlp16_driver_node_ =
        std::make_shared<
        autoware::drivers::velodyne_nodes::VLP16DriverNode>(
        node_options);
    } else if (model == "vlp32c") {
      vlp32_c_driver_node_ =
        std::make_shared<
        autoware::drivers::velodyne_nodes::VLP32CDriverNode>(
        node_options);
    } else if (model == "vls128") {
      vls128_driver_node_ =
        std::make_shared<
        autoware::drivers::velodyne_nodes::VLS128DriverNode>(
        node_options);
    } else {
      throw std::runtime_error("Model " + model + " is not supperted.");
    }
  }
};

}  // namespace velodyne_nodes
}  // namespace drivers
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::drivers::velodyne_nodes::VelodyneCloudWrapperNode)
