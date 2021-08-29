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

#include "interactive_trajectory_spoofer/interactive_trajectory_spoofer_node.hpp"
#include "interactive_trajectory_spoofer/interactive_trajectory_spoofer.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <memory>
#include <string>


namespace autoware
{
namespace interactive_trajectory_spoofer
{
InteractiveTrajectorySpooferNode::InteractiveTrajectorySpooferNode(
  const rclcpp::NodeOptions & node_options)
: Node{"interactive_trajectory_spoofer_node", node_options}
{
  // interactive markers server
  m_server_ptr = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "interactive_trajectory_spoofer", this);
  // original interactive marker and callback to process changes
  m_server_ptr->insert(makeMarker("origin", 0.0, 0.0));  // , std::bind(&InteractiveTrajectorySpooferNode::processMarkerFeedback, std::placeholders::_1, get_logger()));

  // 'commit' changes and send to all clients
  m_server_ptr->applyChanges();
}

void InteractiveTrajectorySpooferNode::processMarkerFeedback(
  const MarkerFeedback::ConstSharedPtr & feedback)
{
  switch (feedback->event_type) {
    case MarkerFeedback::POSE_UPDATE:
      updateControlPoint(feedback->marker_name, feedback->pose);
      break;
  }
}

InteractiveMarker InteractiveTrajectorySpooferNode::makeMarker(
  const std::string & name,
  const float64_t x, const float64_t y)
{
  InteractiveMarker int_marker;
  int_marker.name = name;
  int_marker.header.frame_id = "base_link";
  int_marker.pose.position.x = x;
  int_marker.pose.position.y = y;
  int_marker.scale = 1;

  addControlPoint(name, int_marker.pose);
  return int_marker;
}

void InteractiveTrajectorySpooferNode::addControlPoint(
  const std::string & name,
  const geometry_msgs::msg::Pose & pose)
{
  ControlPoint p;
  p.x = pose.position.x;
  p.y = pose.position.y;
  m_control_points_map[name] = p;
}

void InteractiveTrajectorySpooferNode::updateControlPoint(
  const std::string & name,
  const geometry_msgs::msg::Pose & pose)
{
  m_control_points_map[name].x = pose.position.x;
  m_control_points_map[name].y = pose.position.y;
}
}  // namespace interactive_trajectory_spoofer
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::interactive_trajectory_spoofer::InteractiveTrajectorySpooferNode)
