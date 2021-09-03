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
#include <time_utils/time_utils.hpp>

#include <chrono>
#include <memory>
#include <string>


namespace autoware
{
namespace interactive_trajectory_spoofer
{
InteractiveTrajectorySpooferNode::InteractiveTrajectorySpooferNode(
  const rclcpp::NodeOptions & node_options)
: Node("interactive_trajectory_spoofer_node", node_options), m_tf_buffer(this->get_clock()),
  m_tf_listener(m_tf_buffer)
{
  // Parameters
  float64_t publishing_rate_s = declare_parameter("publishing_rate", 0.1);
  std::string trajectory_topic = declare_parameter("trajectory_topic", "/planning/trajectory");

  // initialize trajectory frame
  m_trajectory.header.frame_id = "map";

  // initialize interactive menu
  m_menu_handler.setCheckState(
    m_menu_handler.insert(
      "Publish trajectory",
      std::bind(
        &InteractiveTrajectorySpooferNode::pubMenuCb, this,
        std::placeholders::_1)),
    interactive_markers::MenuHandler::UNCHECKED);

  m_menu_handler.setCheckState(
    m_menu_handler.insert(
      "Publish preview",
      std::bind(
        &InteractiveTrajectorySpooferNode::previewMenuCb, this,
        std::placeholders::_1)),
    interactive_markers::MenuHandler::UNCHECKED);

  // initialize interactive markers server
  m_server_ptr = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "interactive_trajectory_spoofer", this);

  // Trajectory and trajectory preview publishers
  m_traj_pub = create_publisher<Trajectory>(trajectory_topic, 1);
  m_preview_pub = create_publisher<Trajectory>(
    "interactive_trajectory_spoofer/trajectory_preview",
    1);
  // trajectory publishing on timer
  auto timer_callback = std::bind(&InteractiveTrajectorySpooferNode::timerTrajCb, this);
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<float64_t>(publishing_rate_s));
  m_timer_traj_cb = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period_ns, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(m_timer_traj_cb, nullptr);
}

void InteractiveTrajectorySpooferNode::initializeControlPoints()
{
  geometry_msgs::msg::PointStamped point_baselink;
  // point_baselink.header.stamp = get_clock()->now();
  point_baselink.header.frame_id = "base_link";
  geometry_msgs::msg::PointStamped point_map;
  // interactive markers and callback to process changes
  for (size_t i = 0; i < 6; ++i) {
    const std::string name = "Point" + std::to_string(i);
    point_baselink.point.x = static_cast<decltype(point_baselink.point.x)>(i);
    point_baselink.point.y = 0.0;
    m_tf_buffer.transform(point_baselink, point_map, "map");
    m_server_ptr->insert(
      makeMarker(name, point_map.point.x, point_map.point.y),
      std::bind(
        &InteractiveTrajectorySpooferNode::processMarkerFeedback, this,
        std::placeholders::_1));
    updateControlPoint(name, point_map.point.x, point_map.point.y);
    m_server_ptr->applyChanges();
    m_menu_handler.apply(*m_server_ptr, name);
  }
}


void InteractiveTrajectorySpooferNode::processMarkerFeedback(
  MarkerFeedback::ConstSharedPtr feedback)
{
  switch (feedback->event_type) {
    case MarkerFeedback::POSE_UPDATE:
      updateControlPoint(
        feedback->marker_name, feedback->pose.position.x,
        feedback->pose.position.y);
      break;
  }
}

InteractiveMarker InteractiveTrajectorySpooferNode::makeMarker(
  const std::string & name,
  const float64_t x, const float64_t y)
{
  InteractiveMarker int_marker;
  int_marker.name = name;
  int_marker.header.frame_id = "map";
  int_marker.pose.position.x = x;
  int_marker.pose.position.y = y;
  int_marker.pose.position.z = 4.0;  // put the marker above the vehicle model to make it clickable
  int_marker.scale = 1;

  visualization_msgs::msg::Marker sphere_marker;
  sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
  sphere_marker.scale.x = 0.45;
  sphere_marker.scale.y = 0.45;
  sphere_marker.scale.z = 0.45;
  sphere_marker.color.r = 1.0;
  sphere_marker.color.g = 1.0;
  sphere_marker.color.b = 1.0;
  sphere_marker.color.a = 0.8f;

  visualization_msgs::msg::InteractiveMarkerControl control;
  control.name = name + "_control_xy";
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

  return int_marker;
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

void InteractiveTrajectorySpooferNode::previewMenuCb(MarkerFeedback::ConstSharedPtr feedback)
{
  if (m_publishing_preview) {
    m_menu_handler.setCheckState(
      feedback->menu_entry_id,
      interactive_markers::MenuHandler::UNCHECKED);
    m_publishing_preview = false;
  } else {
    m_menu_handler.setCheckState(
      feedback->menu_entry_id,
      interactive_markers::MenuHandler::CHECKED);
    m_publishing_preview = true;
  }

  m_menu_handler.reApply(*m_server_ptr);
  m_server_ptr->applyChanges();
}

void InteractiveTrajectorySpooferNode::timerTrajCb()
{
  if (m_initialized) {
    if (m_publishing) {
      m_traj_pub->publish(m_trajectory);
    }
    if (m_publishing_preview) {
      m_preview_pub->publish(m_trajectory);
    }
  } else {
    if (m_tf_buffer.canTransform("map", "base_link", tf2::TimePointZero)) {
      initializeControlPoints();
      m_initialized = true;
      RCLCPP_INFO(get_logger(), "Initialized");
    } else {
      RCLCPP_INFO(get_logger(), "Unable to get TF");
    }
  }
}

void InteractiveTrajectorySpooferNode::updateControlPoint(
  const std::string & name, const float64_t x, const float64_t y)
{
  int64_t index;
  std::stringstream sstream(name.substr(5));
  sstream >> index;
  m_curve.updateControlPoint(index, {x, y});
  generateTrajectory();
}

void InteractiveTrajectorySpooferNode::generateTrajectory()
{
  m_trajectory.points.clear();
  TrajectoryPoint tp;
  typedef decltype (TrajectoryPoint::x) ValT;
  constexpr size_t nb_points = Trajectory::CAPACITY;
  const float64_t velocity = 3.0;  // [m/s]
  auto time = rclcpp::Duration(get_clock()->now().nanoseconds());
  auto raw_trajectory = m_curve.cartesianWithHeading(nb_points);
  auto & prev_point = raw_trajectory.front();
  for (const auto & point: raw_trajectory) {
    time = time +
      rclcpp::Duration(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<float64_t>(
          std
          ::hypot(point.x() - prev_point.x(), point.y() - prev_point.y()) / velocity)));
    tp.time_from_start = time;
    tp.x = static_cast<ValT>(point.x());
    tp.y = static_cast<ValT>(point.y());
    tp.heading = motion::motion_common::from_angle(point.z());
    tp.longitudinal_velocity_mps = static_cast<ValT>(velocity);  // TODO set velocity as parameter
    m_trajectory.points.push_back(tp);
  }
}
}  // namespace interactive_trajectory_spoofer
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::interactive_trajectory_spoofer::InteractiveTrajectorySpooferNode)
