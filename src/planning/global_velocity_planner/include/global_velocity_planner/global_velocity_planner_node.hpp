// Copyright 2022 Leo Drive Teknoloji A.Ş.
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
//
// Co-developed by Tier IV, Inc. Apex.AI, Inc. and Leo Drive Teknoloji A.Ş.

/// \copyright Copyright 2022 Leo Drive Teknoloji A.Ş.
/// \file
/// \brief This file defines the global_velocity_planner_node class.

#ifndef GLOBAL_VELOCITY_PLANNER__GLOBAL_VELOCITY_PLANNER_NODE_HPP_
#define GLOBAL_VELOCITY_PLANNER__GLOBAL_VELOCITY_PLANNER_NODE_HPP_


#include <common/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/srv/had_map_service.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_planning_msgs/srv/modify_trajectory.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <global_velocity_planner/global_velocity_planner.hpp>

#include <memory>


namespace autoware
{
namespace global_velocity_planner
{
using motion::motion_common::Real;
using HADMapService = autoware_auto_mapping_msgs::srv::HADMapService;
using autoware_auto_planning_msgs::msg::HADMapRoute;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_auto_planning_msgs::srv::ModifyTrajectory;
using State = autoware_auto_vehicle_msgs::msg::VehicleKinematicState;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::uchar8_t;

/// \class GlobalVelocityPlannerNode
/// \brief Global velocity planner, it is created to test the controllers.
class GLOBAL_VELOCITY_PLANNER_PUBLIC GlobalVelocityPlannerNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit GlobalVelocityPlannerNode(const rclcpp::NodeOptions & options);

  /// \brief print hello
  /// return 0 if successful.

private:
  rclcpp::Subscription<HADMapRoute>::SharedPtr m_route_sub{};
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VehicleKinematicState>::SharedPtr
    state_sub{};
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr
    global_trajectory_pub{};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<HADMapService>::SharedPtr m_map_client;
  // msg cache
  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  std::unique_ptr<GlobalVelocityPlanner> velocity_planner;
  lanelet::LaneletMapPtr m_lanelet_map_ptr;
  HADMapRoute::SharedPtr m_route;
  autoware_auto_vehicle_msgs::msg::VehicleKinematicState pose;
  // callbacks
  void route_callback(const HADMapRoute::SharedPtr & msg);
  void map_response(rclcpp::Client<HADMapService>::SharedFuture future);
  void state_callback(const State::SharedPtr & msg);
  State transform_to_map(const State & state);
  void init();
};
}  // namespace global_velocity_planner
}  // namespace autoware

#endif  // GLOBAL_VELOCITY_PLANNER__GLOBAL_VELOCITY_PLANNER_NODE_HPP_
