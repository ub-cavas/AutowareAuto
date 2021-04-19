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

#ifndef VEHICLE_PARAMETERS_NODE__VEHICLE_PARAMETERS_NODE_HPP_
#define VEHICLE_PARAMETERS_NODE__VEHICLE_PARAMETERS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <vehicle_parameters_node/visibility_control.hpp>

#include <string>
#include <map>
#include <memory>
#include <vector>

namespace autoware
{
namespace vehicle_parameters_node
{

class VEHICLE_PARAMETERS_NODE_PUBLIC VehicleParametersNode : public rclcpp::Node
{
public:
  ///
  /// @brief      default constructor, starts the subscription and publisher.
  ///
  /// @param[in]  options  The node options
  /// @throws     domain_error  if the parameters are specified in a wrong way.
  ///
  explicit VehicleParametersNode(const rclcpp::NodeOptions & options);
};
}  // namespace vehicle_parameters_node
}  // namespace autoware

#endif  // VEHICLE_PARAMETERS_NODE__VEHICLE_PARAMETERS_NODE_HPP_
