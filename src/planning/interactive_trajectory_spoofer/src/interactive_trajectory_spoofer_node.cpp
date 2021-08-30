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
  // Parameters
  float64_t publishing_rate_s = declare_parameter("publishing_rate", 0.1);
  std::string trajectory_topic = declare_parameter("trajectory_topic", "interactive_trajectory");

  // initialize interactive menu
  // m_menu_handler.insert(
  //   "Add point",
  //   std::bind(
  //     &InteractiveTrajectorySpooferNode::addMenuCb, this,
  //     std::placeholders::_1));
  // m_menu_handler.insert("Delete point", &delMenuCb);
  m_menu_handler.setCheckState(
    m_menu_handler.insert(
      "Publish trajectory",
      std::bind(
        &InteractiveTrajectorySpooferNode::pubMenuCb, this,
        std::placeholders::_1)),
    interactive_markers::MenuHandler::UNCHECKED);


  // interactive markers server
  m_server_ptr = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "interactive_trajectory_spoofer", this);
  // original interactive marker and callback to process changes
  m_server_ptr->insert(
    makeMarker("0", 0.0, 0.0),
    std::bind(
      &InteractiveTrajectorySpooferNode::processMarkerFeedback, this,
      std::placeholders::_1));
  m_server_ptr->applyChanges();
  m_menu_handler.apply(*m_server_ptr, "0");

  // Trajectory publisher
  m_traj_pub = create_publisher<Trajectory>(trajectory_topic, 1);

  // trajectory publishing on timer
  auto timer_callback = std::bind(&InteractiveTrajectorySpooferNode::timerTrajCb, this);
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<float64_t>(publishing_rate_s));
  m_timer_traj_cb = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period_ns, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(m_timer_traj_cb, nullptr);
}

void InteractiveTrajectorySpooferNode::processMarkerFeedback(
  MarkerFeedback::ConstSharedPtr feedback)
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

  visualization_msgs::msg::Marker sphere_marker;
  sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
  sphere_marker.scale.x = 0.45;
  sphere_marker.scale.y = 0.45;
  sphere_marker.scale.z = 0.45;
  sphere_marker.color.r = 0.5;
  sphere_marker.color.g = 0.5;
  sphere_marker.color.b = 0.5;
  sphere_marker.color.a = 1.0;

  visualization_msgs::msg::InteractiveMarkerControl control;
  control.name = "move_x_y";
  control.always_visible = true;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  // Orient the control to be aligned with the x,y plane (else the control is for the y,z plane)
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;

  // add control and menu
  control.markers.push_back(sphere_marker);
  int_marker.controls.push_back(control);

  addControlPoint(int_marker.pose);
  return int_marker;
}

void InteractiveTrajectorySpooferNode::addMenuCb(MarkerFeedback::ConstSharedPtr /*feedback*/)
{
  const std::string name = std::to_string(m_control_points.size());
  m_server_ptr->insert(
    makeMarker(name, m_control_points.back().x + 1.0, m_control_points.back().y + 1.0),
    std::bind(
      &InteractiveTrajectorySpooferNode::processMarkerFeedback, this,
      std::placeholders::_1));
  m_server_ptr->applyChanges();
  m_menu_handler.apply(*m_server_ptr, name);
}
void InteractiveTrajectorySpooferNode::pubMenuCb(MarkerFeedback::ConstSharedPtr feedback)
{
  if (m_publishing) {
    m_menu_handler.setCheckState(
      feedback->menu_entry_id,
      interactive_markers::MenuHandler::UNCHECKED);
    m_publishing = false;
  } else {
    m_menu_handler.setCheckState(
      feedback->menu_entry_id,
      interactive_markers::MenuHandler::CHECKED);
    m_publishing = true;
  }

  m_menu_handler.reApply(*m_server_ptr);
  m_server_ptr->applyChanges();
}

void InteractiveTrajectorySpooferNode::timerTrajCb()
{
  if (m_publishing) {
    Trajectory traj;
    traj.header.frame_id = "map";
    TrajectoryPoint p;
    for (ControlPoint cp: m_control_points) {
      p.x = static_cast<decltype(p.x)>(cp.x);
      p.y = static_cast<decltype(p.y)>(cp.y);
      traj.points.push_back(p);
    }
    m_traj_pub->publish(traj);
  }
}

void InteractiveTrajectorySpooferNode::deleteControlPoint(const std::string & name)
{
  size_t index;
  std::stringstream sstream(name);
  sstream >> index;
  m_control_points.resize(index - 1);
}

void InteractiveTrajectorySpooferNode::addControlPoint(const geometry_msgs::msg::Pose & pose)
{
  ControlPoint p;
  p.x = pose.position.x;
  p.y = pose.position.y;
  m_control_points.push_back(p);
}

void InteractiveTrajectorySpooferNode::updateControlPoint(
  const std::string & name,
  const geometry_msgs::msg::Pose & pose)
{
  size_t index;
  std::stringstream sstream(name);
  sstream >> index;
  m_control_points[index].x = pose.position.x;
  m_control_points[index].y = pose.position.y;
}
}  // namespace interactive_trajectory_spoofer
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::interactive_trajectory_spoofer::InteractiveTrajectorySpooferNode)
