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

#ifndef MEASUREMENT_TRANSFORMER_NODES__POSE_WITH_COV_CHILD_FRAME_TRANSFORMER_NODE_HPP_
#define MEASUREMENT_TRANSFORMER_NODES__POSE_WITH_COV_CHILD_FRAME_TRANSFORMER_NODE_HPP_

#include <measurement_transformer_nodes/measurement_transformer_node_base.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::TransformStamped;

namespace autoware
{
namespace measurement_transformer_nodes
{

/// \class PoseWithCovChildFrameTransformerNode
/// \brief ROS 2 Node for hello world.
class MEASUREMENT_TRANSFORMER_NODES_PUBLIC PoseWithCovChildFrameTransformerNode
  : public ChildFrameTransformerNode<PoseWithCovarianceStamped>
{
public:
  using ParentT = ChildFrameTransformerNode<PoseWithCovarianceStamped>;

  /// \brief Default constructor
  explicit PoseWithCovChildFrameTransformerNode(const rclcpp::NodeOptions & options);

private:
  /// \brief Concrete function to convert a TransformStamped to a measurement
  /// \param tf The TransformStamped to convert to a measurement
  /// \param orig_measurement The original measurement from which to create the new
  /// \returns A measurement converted from a TransformStamped
  PoseWithCovarianceStamped transform_to_measurement(
    const TransformStamped & tf,
    const PoseWithCovarianceStamped & orig_measurement) override;

  /// \brief Concrete function to convert a measurement to a TransformStamped
  /// \param measurement The measurement to convert to a TransformStamped
  /// \returns A TransformStamped converted from a measurement
  TransformStamped measurement_to_transform(
    const PoseWithCovarianceStamped & measurement) override;

  /// \brief Concrete function for applying a transform to a measurement
  /// \param[in] measurement_in The measurement to be transformed
  /// \param[out] measurement_out The result of the transform
  /// \param[in] tf The transform to apply
  void apply_transform(
    const PoseWithCovarianceStamped & measurement_in,
    PoseWithCovarianceStamped & measurement_out,
    const TransformStamped & tf) override;
};
}  // namespace measurement_transformer_nodes
}  // namespace autoware

#endif  // MEASUREMENT_TRANSFORMER_NODES__POSE_WITH_COV_CHILD_FRAME_TRANSFORMER_NODE_HPP_
