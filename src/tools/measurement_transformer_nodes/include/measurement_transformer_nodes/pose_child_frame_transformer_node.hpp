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

#ifndef MEASUREMENT_TRANSFORMER_NODES__POSE_CHILD_FRAME_TRANSFORMER_NODE_HPP_
#define MEASUREMENT_TRANSFORMER_NODES__POSE_CHILD_FRAME_TRANSFORMER_NODE_HPP_

#include <measurement_transformer_nodes/measurement_transformer_node_base.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;

namespace autoware
{
namespace measurement_transformer_nodes
{

/// \class PoseChildFrameTransformerNode
/// \brief ROS 2 Node for hello world.
class MEASUREMENT_TRANSFORMER_NODES_PUBLIC PoseChildFrameTransformerNode
  : public ChildFrameTransformerNode<PoseStamped>
{
public:
  using ParentT = ChildFrameTransformerNode<PoseStamped>;

  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit PoseChildFrameTransformerNode(const rclcpp::NodeOptions & options);

private:
  PoseStamped transform_to_measurement(const TransformStamped & tf) override;
  TransformStamped measurement_to_transform(const PoseStamped & measurement) override;
};
}  // namespace measurement_transformer_nodes
}  // namespace autoware

#endif  // MEASUREMENT_TRANSFORMER_NODES__POSE_CHILD_FRAME_TRANSFORMER_NODE_HPP_
