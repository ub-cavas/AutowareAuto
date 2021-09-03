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
/// \brief This file defines the InteractiveTrajectorySpooferNode class.

#ifndef INTERACTIVE_TRAJECTORY_SPOOFER__INTERACTIVE_TRAJECTORY_SPOOFER_NODE_HPP_
#define INTERACTIVE_TRAJECTORY_SPOOFER__INTERACTIVE_TRAJECTORY_SPOOFER_NODE_HPP_

#include <interactive_trajectory_spoofer/bezier.hpp>
#include <interactive_trajectory_spoofer/visibility_control.hpp>

#include "autoware_auto_msgs/msg/trajectory.hpp"
#include "common/types.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "motion_common/motion_common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace autoware
{
namespace interactive_trajectory_spoofer
{
using InteractiveMarker = visualization_msgs::msg::InteractiveMarker;
using MarkerFeedback = visualization_msgs::msg::InteractiveMarkerFeedback;
using Trajectory = autoware_auto_msgs::msg::Trajectory;
using std::placeholders::_1;
using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;

/// \class InteractiveTrajectorySpooferNode
/// \brief ROS 2 Node for creating interactive fake trajectories
class INTERACTIVE_TRAJECTORY_SPOOFER_PUBLIC InteractiveTrajectorySpooferNode
  : public rclcpp::Node
{
public:
  explicit InteractiveTrajectorySpooferNode(const rclcpp::NodeOptions & node_options);

  /**
   * @brief callback to handle changes in the interactive markers
   * @param [in] feedback feedback message
   */
  void processMarkerFeedback(MarkerFeedback::ConstSharedPtr feedback);

private:
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> m_server_ptr;
  interactive_markers::MenuHandler m_menu_handler;
  Bezier m_curve;
  Trajectory m_trajectory;

  rclcpp::Publisher<Trajectory>::SharedPtr m_traj_pub;
  rclcpp::Publisher<Trajectory>::SharedPtr m_preview_pub;
  rclcpp::TimerBase::SharedPtr m_timer_traj_cb;

  // Buffer, listener, and transform to convert from base_link to map
  tf2_ros::Buffer m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;


  bool8_t m_publishing = false;
  bool8_t m_publishing_preview = false;
  bool8_t m_initialized = false;

  /**
   * @brief initialize the control points of the curve and corresponding interactive markers
   */
  void initializeControlPoints();

  /**
   * @brief create an interactive marker with the given name and (x,y) coordinates
   * @param [in] name name of the marker
   * @param [in] x x position of the marker
   * @param [in] y y position of the marker
   * @return created interactive marker
   */
  InteractiveMarker makeMarker(const std::string & name, const float64_t x, const float64_t y);

  /**
   * @brief update the given control point with the given pose
   * @param [in] name name of the interactive marker associated with the control point
   * @param [in] x x position of the map
   * @param [in] y y position of the map
   */
  void updateControlPoint(const std::string & name, const float64_t x, const float64_t y);

  /**
   * @brief callback for the checkbox to publish or not the trajectory
   * @param [in] feedback menu feedback
   */
  void pubMenuCb(MarkerFeedback::ConstSharedPtr feedback);

  /**
   * @brief callback for the checkbox to publish or not the trajectory preview
   * @param [in] feedback menu feedback
   */
  void previewMenuCb(MarkerFeedback::ConstSharedPtr feedback);

  /**
   * @brief generate the trajectory for the current curve
   */
  void generateTrajectory();

  /**
   * @brief timer callback to publish the trajectory
   */
  void timerTrajCb();
};

}  // namespace interactive_trajectory_spoofer
}  // namespace autoware

#endif  // INTERACTIVE_TRAJECTORY_SPOOFER__INTERACTIVE_TRAJECTORY_SPOOFER_NODE_HPP_
