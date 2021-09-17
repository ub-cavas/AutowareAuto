// Copyright 2020 Tier IV, Inc.
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

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef FREESPACE_PLANNER__FREESPACE_PLANNER_HPP_
#define FREESPACE_PLANNER__FREESPACE_PLANNER_HPP_

#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <freespace_planner/visibility_control.hpp>
#include <astar_search/astar_search.hpp>

#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <motion_common/motion_common.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_planning_msgs/msg/route.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <autoware_auto_msgs/srv/planner_costmap.hpp>
#include <autoware_auto_msgs/action/plan_trajectory.hpp>

struct NodeParam
{
  double waypoints_velocity;  // constant velocity on planned waypoints [km/h]
  double update_rate;         // replanning and publishing rate [Hz]
  double th_arrived_distance_m;
  double th_stopped_time_sec;
  double th_stopped_velocity_mps;
  double th_course_out_distance_m;
  bool replan_when_obstacle_found;
  bool replan_when_course_out;
};

class FreespacePlannerNode : public rclcpp::Node
{
public:
  explicit FreespacePlannerNode(const rclcpp::NodeOptions & node_options);

private:
  using PlanTrajectoryAction = autoware_auto_msgs::action::PlanTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlanTrajectoryAction>;
  using PlannerCostmap = autoware_auto_msgs::srv::PlannerCostmap;

  // ros communication
  rclcpp::Client<PlannerCostmap>::SharedPtr map_client_;
  rclcpp_action::Server<PlanTrajectoryAction>::SharedPtr plan_trajectory_srv_;
  rclcpp::Publisher<autoware_auto_msgs::msg::Trajectory>::SharedPtr trajectory_debug_pub_;

  // TODO probably will be deleted
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // params
  NodeParam node_param_;
  AstarParam astar_param_;

  // variables
  std::unique_ptr<AstarSearch> astar_;
  std::shared_ptr<GoalHandle> goal_handle_{nullptr};
  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  autoware_auto_msgs::msg::Trajectory trajectory_;
  nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_;

  // callbacks
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const PlanTrajectoryAction::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);
  void onOccupancyGrid(rclcpp::Client<PlannerCostmap>::SharedFuture future);

  // functions
  void reset();
  void planTrajectory();
  geometry_msgs::msg::TransformStamped getTransform(
    const std::string & from, const std::string & to);
};

#endif  // FREESPACE_PLANNER__FREESPACE_PLANNER_HPP_
