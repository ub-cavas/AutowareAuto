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
/// \brief This file defines the global_velocity_planner class.

#ifndef GLOBAL_VELOCITY_PLANNER__GLOBAL_VELOCITY_PLANNER_HPP_
#define GLOBAL_VELOCITY_PLANNER__GLOBAL_VELOCITY_PLANNER_HPP_

#include <common/types.hpp>
#include <global_velocity_planner/visibility_control.hpp>
#include <motion_common/config.hpp>
#include <motion_common/motion_common.hpp>

#include <autoware_auto_mapping_msgs/srv/had_map_service.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_report.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
namespace autoware
{
/// \brief TODO(berkay): Document namespaces!
namespace global_velocity_planner
{
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;
using autoware_auto_planning_msgs::msg::HADMapRoute;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using lanelet::LaneletMapConstPtr;
using motion::motion_common::VehicleConfig;
using State = autoware_auto_vehicle_msgs::msg::VehicleKinematicState;
/**
 * @brief it is waypoint variable, any point should have speed limit (read from map),
 * curvature which is calculated in calculate_curvatures function,
 * and Trajectory point which is taken from lanelets in route.
 */
struct GLOBAL_VELOCITY_PLANNER_PUBLIC point
{
  TrajectoryPoint point;
  float32_t speed_limit;
  float32_t curvature;
};
/**
 * @brief It is planner configuration parameters.
 * @param [in] trajectory_resolution represents distance between waypoints.
 * @param [in] lateral_acceleration it is the maximum lateral acceleration in high curvature points.
 * @param [in] longitudinal_acceleration represents maximum longitudinal acceleration.
 */
struct GLOBAL_VELOCITY_PLANNER_PUBLIC GlobalVelocityPlannerConfig
{
  float32_t trajectory_resolution;
  float32_t lateral_acceleration;
  float32_t longitudinal_acceleration;
};
size_t GLOBAL_VELOCITY_PLANNER_PUBLIC get_closest_lanelet(
  const lanelet::ConstLanelets & lanelets,
  const TrajectoryPoint & point);
float32_t GLOBAL_VELOCITY_PLANNER_PUBLIC find_acceleration(
  const TrajectoryPoint & p1,
  const TrajectoryPoint & p2);
float32_t GLOBAL_VELOCITY_PLANNER_PUBLIC find_velocity(
  const TrajectoryPoint & p1,
  const TrajectoryPoint & p2,
  float32_t longitudinal_acceleration);

class GLOBAL_VELOCITY_PLANNER_PUBLIC GlobalVelocityPlanner
{
public:
  explicit GlobalVelocityPlanner(
    const VehicleConfig & vehicle_param, const GlobalVelocityPlannerConfig & planner_config);
  std::shared_ptr<std::vector<point>> way_points;
  bool8_t is_route_ready = false;
  Trajectory trajectory;
  VehicleConfig vehicle_param;
  GlobalVelocityPlannerConfig velocity_planner_config;
  // functions
  /**
 * @brief Set the route by using output of global planner and map data
   */
  void set_route(const HADMapRoute & route, const lanelet::LaneletMapPtr & lanelet_map_ptr);
  /**
 * @brief Checks the route empty or not
   */
  bool8_t is_route_empty();
  /**
 * @brief Checks the route is over or not
   */
  bool8_t is_route_over();

  void clear_route();
  /**
 * @brief It calculates waypoints by using the map data and global route
   * Calculated waypoints have steering angle, velocity, acceleration, orientation with respect to
   * path's characteristic.
   */
  void calculate_waypoints();
  /**
 * @brief It calculates trajectory wrt waypoints. It cuts the trajectory wrt vehicle's state.
   * If trajectory size is smaller than 50 and there are more than 50 waypoints, need_trajectory
   * returns true and all trajectory is calculated again.
   * It can be used in both reverse and forward driving.
   */
  void calculate_trajectory(const State & pose);

private:
  /**
 * @brief It calculates the curvature of all points by using neighbour points. The first and last
   * points' curvature is NaN.
   * It uses Menger Curvature: https://en.wikipedia.org/wiki/Menger_curvature
   */
  void calculate_curvatures();
  /**
 * @brief It determines the upper and lower bound of longitudinal velocity with respect to maximum
   * lateral acceleration.
   */
  void vel_wrt_lateral_acceleration();
  /**
 * @brief Calculates other points' velocity by using output of vel_wrt_lateral_acceleration and
   * maximum longitudinal velocity.
   */
  void vel_wrt_longitudinal_acceleration();
  /**
 * @brief Sets steering angle by using waypoints.
   */
  void set_steering_angle(point & pt);
  /**
 * @brief Sets orientation by using waypoints.
   */
  void set_orientation(size_t i);
  /**
 * @brief It calculates the timing of trajectories with respect to vehicle's longitudinal velocity
   */
  void set_time_from_start();
  /**
 * @brief Checks the vehichle needs trajectory or not. If remaining trajectory lower than 50 and
   * remaining waypoints more than 50, It returns true.
   */
  bool8_t need_trajectory();
  /**
 * @brief It gets closest waypoint index wrt vehicle's pose.
   */
  // TODO(berkay6): remove get_closest_index use common function motion_common::findNearestIndex
  size_t get_closest_index(const State & pose);
  // variables

  std::shared_ptr<HADMapRoute> route;
  lanelet::LaneletMapPtr map;
  size_t last_point;
};
/// \brief TODO(berkay): Document your functions
}  // namespace global_velocity_planner
}  // namespace autoware

#endif  // GLOBAL_VELOCITY_PLANNER__GLOBAL_VELOCITY_PLANNER_HPP_
