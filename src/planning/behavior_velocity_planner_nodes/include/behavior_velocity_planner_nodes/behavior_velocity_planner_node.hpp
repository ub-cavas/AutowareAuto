// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the behavior_velocity_planner_node class.

#ifndef BEHAVIOR_VELOCITY_PLANNER_NODES__BEHAVIOR_VELOCITY_PLANNER_NODE_HPP_
#define BEHAVIOR_VELOCITY_PLANNER_NODES__BEHAVIOR_VELOCITY_PLANNER_NODE_HPP_

#include <behavior_velocity_planner/behavior_velocity_planner.hpp>
#include <behavior_velocity_planner_nodes/planner_data.hpp>
#include <behavior_velocity_planner_nodes/planner_manager.hpp>
#include <behavior_velocity_planner_nodes/visibility_control.hpp>

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/srv/had_map_service.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/order_movement.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <vector>
#include <string>

namespace autoware
{
namespace planning
{
namespace behavior_velocity_planner_nodes
{
class BEHAVIOR_VELOCITY_PLANNER_NODES_PUBLIC BehaviorVelocityPlannerNode : public rclcpp::Node
{
public:
  explicit BehaviorVelocityPlannerNode(const rclcpp::NodeOptions & options);
  void request_osm_binary_map();

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_diagnostic_status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_debug_;

  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr
    sub_predicted_objects_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_no_ground_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VehicleKinematicState>::SharedPtr
    sub_vehicle_state_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr
    sub_had_map_bin_lanelet_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>::SharedPtr
    sub_path_with_lane_id_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::OrderMovement>::SharedPtr
    sub_order_movement_crosswalk_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::OrderMovement>::SharedPtr
    sub_order_movement_intersection_;

  rclcpp::Client<autoware_auto_mapping_msgs::srv::HADMapService>::SharedPtr map_client_;

  PlannerData planner_data_;
  BehaviorVelocityPlannerManager planner_manager_;

  double forward_path_length_;
  double backward_path_length_;

  void callback_predicted_objects(
    const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg_in);
  void callback_cloud_no_ground(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_in);
  void callback_vehicle_state(
    const autoware_auto_vehicle_msgs::msg::VehicleKinematicState::ConstSharedPtr msg_in);
  void callback_path_with_lane_id(
    const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg_in);
  void callback_order_movement_crosswalk(
    const autoware_auto_planning_msgs::msg::OrderMovement::ConstSharedPtr msg_in);
  void callback_order_movement_intersection(
    const autoware_auto_planning_msgs::msg::OrderMovement::ConstSharedPtr msg_in);

  void processs_had_map_bin_lanelet(const autoware_auto_mapping_msgs::msg::HADMapBin & had_map_bin);

  bool is_data_ready();
  autoware_auto_planning_msgs::msg::Trajectory path_to_trajectory(
    const autoware_auto_planning_msgs::msg::Path & path);
  void set_steering_angle(
    std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & trajectory_points);
  float distance2d(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);
  float calculate_curvature(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    const geometry_msgs::msg::Point & p3);
  static void set_time_from_start(
    std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & trajectory_points);
  void publish_debug_marker(const autoware_auto_planning_msgs::msg::Path & path);
  autoware_auto_planning_msgs::msg::Path transform_path(
    const autoware_auto_planning_msgs::msg::Path & path,
    const std::string & source_frame_id,
    const std::string & target_frame_id);
};
}  // namespace behavior_velocity_planner_nodes
}  // namespace planning
}  // namespace autoware

#endif  // BEHAVIOR_VELOCITY_PLANNER_NODES__BEHAVIOR_VELOCITY_PLANNER_NODE_HPP_
