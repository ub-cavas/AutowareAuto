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

#include "measurement_transformer_nodes/pose_with_cov_parent_frame_transformer_node.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace autoware
{
namespace measurement_transformer_nodes
{

PoseWithCovParentFrameTransformerNode::PoseWithCovParentFrameTransformerNode(
  const rclcpp::NodeOptions & options)
: ParentFrameTransformerNode<PoseWithCovarianceStamped>(
    "pose_parent_frame_transformer",
    "pose_stamped_in",
    "pose_stamped_out",
    options)
{
}

void PoseWithCovParentFrameTransformerNode::apply_transform(
  const PoseWithCovarianceStamped & measurement_in,
  PoseWithCovarianceStamped & measurement_out,
  const TransformStamped & tf)
{
  tf2::doTransform(measurement_in, measurement_out, tf);
}

}  // namespace measurement_transformer_nodes
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::measurement_transformer_nodes::PoseWithCovParentFrameTransformerNode)
