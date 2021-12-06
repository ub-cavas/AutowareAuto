// Copyright 2020 The Autoware Foundation
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

/// \copyright Copyright 2020 The Autoware Foundation
/// \file
/// \brief This file defines the behavior_planner_nodes_node class.

#ifndef BEHAVIOR_PLANNER_NODES__BEHAVIOR_PLANNER_NODE_HPP_
#define BEHAVIOR_PLANNER_NODES__BEHAVIOR_PLANNER_NODE_HPP_

#include <behavior_planner_nodes/behavior_planner_node.hpp>
#include <behavior_planner_nodes/visibility_control.hpp>

// rclcpp headers
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// autoware packages
#include <common/types.hpp>
#include <autoware_auto_planning_msgs/action/plan_trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_mapping_msgs/srv/had_map_service.hpp>
#include <autoware_auto_planning_msgs/srv/modify_trajectory.hpp>
#include <behavior_planner/behavior_planner.hpp>

//  Other ROS packages
#include <lanelet2_core/LaneletMap.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose_array.hpp>

// others
#include <string>
#include <memory>

namespace autoware
{
namespace behavior_planner_nodes
{
using PlanTrajectoryAction = autoware_auto_planning_msgs::action::PlanTrajectory;
using PlanTrajectoryGoalHandle = rclcpp_action::ClientGoalHandle<PlanTrajectoryAction>;
using HADMapService = autoware_auto_mapping_msgs::srv::HADMapService;
using autoware_auto_planning_msgs::srv::ModifyTrajectory;
using geometry_msgs::msg::Pose;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_auto_planning_msgs::msg::HADMapRoute;
using autoware_auto_vehicle_msgs::msg::VehicleStateCommand;
using autoware_auto_vehicle_msgs::msg::VehicleStateReport;
using autoware_auto_vehicle_msgs::msg::GearReport;
using State = autoware_auto_vehicle_msgs::msg::VehicleKinematicState;
using autoware::behavior_planner::PlannerType;
using autoware::behavior_planner::RouteWithType;

using autoware::common::types::bool8_t;
using autoware::common::types::uchar8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

/// \class BehaviorPlannerNode
/// \brief ROS 2 Node for wrapping behavior planner
class BEHAVIOR_PLANNER_NODES_PUBLIC BehaviorPlannerNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts the planner
  /// \param[in] options name of the node for rclcpp internals
  explicit BehaviorPlannerNode(const rclcpp::NodeOptions & options);

private:
  //  ROS Interface
  rclcpp_action::Client<PlanTrajectoryAction>::SharedPtr m_lane_planner_client;
  rclcpp_action::Client<PlanTrajectoryAction>::SharedPtr m_parking_planner_client;
  rclcpp::Client<HADMapService>::SharedPtr m_map_client;
  // May be nullptr if disabled
  rclcpp::Client<ModifyTrajectory>::SharedPtr m_modify_trajectory_client;
  rclcpp::Subscription<State>::SharedPtr m_ego_state_sub{};
  rclcpp::Subscription<HADMapRoute>::SharedPtr m_route_sub{};
  rclcpp::Subscription<Trajectory>::SharedPtr m_lane_trajectory_sub{};
  rclcpp::Subscription<Trajectory>::SharedPtr m_parking_trajectory_sub{};
  rclcpp::Subscription<GearReport>::SharedPtr m_gear_report_sub{};
  rclcpp::Publisher<Trajectory>::SharedPtr m_trajectory_pub{};
  rclcpp::Publisher<Trajectory>::SharedPtr m_debug_trajectory_pub{};
  rclcpp::Publisher<Trajectory>::SharedPtr m_debug_checkpoints_pub{};
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_debug_global_path_pub{};
  rclcpp::Publisher<HADMapRoute>::SharedPtr m_debug_subroute_pub{};
  rclcpp::Publisher<VehicleStateCommand>::SharedPtr m_vehicle_state_command_pub{};

  //  planner
  std::unique_ptr<behavior_planner::BehaviorPlanner> m_planner;

  // msg cache
  lanelet::LaneletMapPtr m_lanelet_map_ptr;
  HADMapRoute::SharedPtr m_route;
  State m_ego_state;
  uchar8_t m_current_gear;

  // bools to manage states
  bool8_t m_requesting_trajectory;
  bool8_t m_log_goal_reached{true};

  // transforms
  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

  // callbacks
  void on_ego_state(const State::SharedPtr & msg);
  void on_route(const HADMapRoute::SharedPtr & msg);
  void on_lane_trajectory(const Trajectory::SharedPtr & msg);
  void on_parking_trajectory(const Trajectory::SharedPtr & msg);
  void on_gear_report(const GearReport::SharedPtr & msg);
  void map_response(rclcpp::Client<HADMapService>::SharedFuture future);
  void modify_trajectory_response(rclcpp::Client<ModifyTrajectory>::SharedFuture future);
  void clear_trajectory_cache();

  void goal_response_callback(std::shared_future<PlanTrajectoryGoalHandle::SharedPtr> future);
  void feedback_callback(
    PlanTrajectoryGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const PlanTrajectoryAction::Feedback> feedback);
  void result_callback(const PlanTrajectoryGoalHandle::WrappedResult & result);

  // other functions
  void init();
  Trajectory refine_trajectory(const State & ego_state, const Trajectory & input);
  State transform_to_map(const State & state);
  void request_trajectory(const RouteWithType & route_with_type);
  void visualize_global_path(const HADMapRoute & route);
};
}  // namespace behavior_planner_nodes
}  // namespace autoware

#endif  // BEHAVIOR_PLANNER_NODES__BEHAVIOR_PLANNER_NODE_HPP_
