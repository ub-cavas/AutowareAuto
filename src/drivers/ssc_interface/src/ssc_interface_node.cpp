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

#include <memory>

//lint -e537 NOLINT  // cpplint vs pclint
#include <string>

namespace ssc_interface
{

SscInterfaceNode::SscInterfaceNode(const rclcpp::NodeOptions & options)
: VehicleInterfaceNode{"ssc_interface", options}
{
  set_interface(std::make_unique<SscInterface>(*this));
}

}  // namespace ssc_interface

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(ssc_interface::SscInterfaceNode)
