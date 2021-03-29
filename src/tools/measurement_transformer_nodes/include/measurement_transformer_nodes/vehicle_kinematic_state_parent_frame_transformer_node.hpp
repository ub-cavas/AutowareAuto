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
/// \brief This file defines the measurement_transformer_nodes_node class.

#ifndef MEASUREMENT_TRANSFORMER_NODES__VEHICLE_KINEMATIC_STATE_PARENT_FRAME_TRANSFORMER_NODE_HPP_
#define MEASUREMENT_TRANSFORMER_NODES__VEHICLE_KINEMATIC_STATE_PARENT_FRAME_TRANSFORMER_NODE_HPP_

#include <measurement_transformer_nodes/measurement_transformer_node_base.hpp>

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>

using autoware_auto_msgs::msg::VehicleKinematicState;
using geometry_msgs::msg::TransformStamped;

namespace autoware
{
namespace measurement_transformer_nodes
{

/// \class VehicleKinematicStateParentFrameTransformerNode
/// \brief Node to transform a VehicleKinematicState parent frame
class MEASUREMENT_TRANSFORMER_NODES_PUBLIC VehicleKinematicStateParentFrameTransformerNode
  : public ParentFrameTransformerNode<VehicleKinematicState>
{
public:
  using ParentT = ParentFrameTransformerNode<VehicleKinematicState>;

  /// \brief Default constructor
  explicit VehicleKinematicStateParentFrameTransformerNode(const rclcpp::NodeOptions & options);

private:
  /// \brief Concrete function for applying a transform to a measurement
  /// \param[in] measurement_in The measurement to be transformed
  /// \param[out] measurement_out The result of the transform
  /// \param[in] tf The transform to apply
  void apply_transform(
    const VehicleKinematicState & measurement_in,
    VehicleKinematicState & measurement_out,
    const TransformStamped & tf) override;
};
}  // namespace measurement_transformer_nodes
}  // namespace autoware

#endif  // MEASUREMENT_TRANSFORMER_NODES__VEHICLE_KINEMATIC_STATE_PARENT_FRAME_TRANSFORMER_NODE_HPP_
