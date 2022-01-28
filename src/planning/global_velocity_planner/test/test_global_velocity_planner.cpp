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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.


#include <memory>


#include "gtest/gtest.h"

#define private public  // to test the private functions
#include "global_velocity_planner/global_velocity_planner.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware_auto_mapping_msgs::msg::HADMapSegment;
using autoware_auto_mapping_msgs::msg::MapPrimitive;
using autoware_auto_planning_msgs::msg::HADMapRoute;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using motion::motion_common::VehicleConfig;
using State = autoware_auto_vehicle_msgs::msg::VehicleKinematicState;

struct GlobalVelocityPlannerConfig
{
  float32_t trajectory_resolution;
  float32_t lateral_acceleration;
  float32_t longitudinal_acceleration;
};

struct point
{
  TrajectoryPoint point;
  float32_t speed_limit;
  float32_t curvature;
};

// returns a map with a lane has given number of points(n_points)
// length of the lane will be n_points meters in y direction
lanelet::LaneletMapPtr getALaneletMapWithLaneId(
  const lanelet::Id & id, const float64_t velocity, const size_t n_points)
{
  lanelet::Points3d right_points, left_points, center_points;
  constexpr float64_t resolution = 1.0;
  for (size_t i = 0; i < n_points; i++) {
    const auto y = resolution * static_cast<float64_t>(i);
    left_points.push_back(lanelet::Point3d(lanelet::utils::getId(), -1, y, 0));
    right_points.push_back(lanelet::Point3d(lanelet::utils::getId(), 1, y, 0));
    center_points.push_back(lanelet::Point3d(lanelet::utils::getId(), 0, y, 0));
  }
  lanelet::LineString3d ls1(lanelet::utils::getId(), left_points);
  lanelet::LineString3d ls2(lanelet::utils::getId(), right_points);
  lanelet::LineString3d ls3(lanelet::utils::getId(), center_points);

  lanelet::Lanelet ll(id, ls1, ls2);
  ll.setCenterline(ls3);
  ll.setAttribute(lanelet::AttributeName::SpeedLimit, velocity);

  return lanelet::utils::createMap({ll});
}

HADMapRoute getARoute(const int64_t lane_id, const float32_t length)
{
  HADMapRoute had_map_route;
  had_map_route.start_pose.position.x = 0;
  had_map_route.start_pose.position.y = 0;
  had_map_route.goal_pose.position.x = 0;
  had_map_route.goal_pose.position.y = length;

  MapPrimitive primitive;
  primitive.id = lane_id;
  HADMapSegment segment;
  segment.preferred_primitive_id = primitive.id;
  had_map_route.segments.push_back(segment);
  had_map_route.segments.front().primitives.push_back(primitive);

  return had_map_route;
}


class GlobalVelocityPlannerTest : public ::testing::Test
{
public:
  std::shared_ptr<autoware::global_velocity_planner::GlobalVelocityPlanner> velocity_planner_ptr;
  GlobalVelocityPlannerTest()
  {
    const VehicleConfig vehicle_param{
      1.0F,     // cg_to_front_m:
      1.0F,     // cg_to_rear_m:
      0.1F,     // front_corner_stiffness:
      0.1F,     // rear_corner_stiffness:
      1500.0F,  // mass_kg:
      12.0F,    // yaw_inertia_kgm2:
      2.0F,     // width_m:
      0.5F,     // front_overhang_m:
      0.5F      // rear_overhang_m:
    };

    const autoware::global_velocity_planner::GlobalVelocityPlannerConfig planner_config{
      0.5F, 0.3F, 0.5F};
    velocity_planner_ptr =
      std::make_shared<autoware::global_velocity_planner::GlobalVelocityPlanner>(
      vehicle_param, planner_config);
  }
};
// Initialize the waypoints


TEST_F(GlobalVelocityPlannerTest, waypoint_test)
{
  autoware::global_velocity_planner::point p0, p1, p2, p3, p4;
  p0.point.pose.position.x = 0;
  p0.point.pose.position.y = 0;
  p0.speed_limit = 1.5;
  p1.point.pose.position.x = 5;
  p1.point.pose.position.y = 5;
  p1.speed_limit = 1.5;
  p2.point.pose.position.x = 10;
  p2.point.pose.position.y = 5;
  p2.speed_limit = 1.5;
  p3.point.pose.position.x = 10;
  p3.point.pose.position.y = 10;
  p3.speed_limit = 1.5;
  p4.point.pose.position.x = 5;
  p4.point.pose.position.y = 15;
  p4.speed_limit = 1.5;
  velocity_planner_ptr->way_points->clear();
  velocity_planner_ptr->way_points->push_back(p0);
  velocity_planner_ptr->way_points->push_back(p1);
  velocity_planner_ptr->way_points->push_back(p2);
  velocity_planner_ptr->way_points->push_back(p3);
  velocity_planner_ptr->way_points->push_back(p4);
  velocity_planner_ptr->calculate_curvatures();

  ASSERT_TRUE(std::isnan(velocity_planner_ptr->way_points->at(0).curvature));
  ASSERT_FLOAT_EQ(velocity_planner_ptr->way_points->at(1).curvature, 0.126491106F);
  ASSERT_FLOAT_EQ(velocity_planner_ptr->way_points->at(2).curvature, 0.282842712F);
  ASSERT_FLOAT_EQ(velocity_planner_ptr->way_points->at(3).curvature, 0.126491106F);
  ASSERT_TRUE(std::isnan(velocity_planner_ptr->way_points->at(4).curvature));


  // velocities with respect to lateral acceleration

  velocity_planner_ptr->vel_wrt_lateral_acceleration();
  ASSERT_FLOAT_EQ(velocity_planner_ptr->way_points->at(0).point.longitudinal_velocity_mps, 0.0F);
  ASSERT_FLOAT_EQ(velocity_planner_ptr->way_points->at(1).point.longitudinal_velocity_mps, 1.5F);
  ASSERT_FLOAT_EQ(
    velocity_planner_ptr->way_points->at(2).point.longitudinal_velocity_mps, 1.029883573F);
  ASSERT_FLOAT_EQ(velocity_planner_ptr->way_points->at(3).point.longitudinal_velocity_mps, 1.5F);
  ASSERT_FLOAT_EQ(velocity_planner_ptr->way_points->at(4).point.longitudinal_velocity_mps, 0.0F);

  // velocities with respect to longitudinal acceleration

  velocity_planner_ptr->vel_wrt_longitudinal_acceleration();
  ASSERT_FLOAT_EQ(velocity_planner_ptr->way_points->at(0).point.longitudinal_velocity_mps, 0.0F);
  ASSERT_FLOAT_EQ(velocity_planner_ptr->way_points->at(1).point.longitudinal_velocity_mps, 1.5F);
  ASSERT_FLOAT_EQ(
    velocity_planner_ptr->way_points->at(2).point.longitudinal_velocity_mps, 1.029883573F);
  ASSERT_FLOAT_EQ(velocity_planner_ptr->way_points->at(3).point.longitudinal_velocity_mps, 1.5F);
  ASSERT_FLOAT_EQ(velocity_planner_ptr->way_points->at(4).point.longitudinal_velocity_mps, 0.0F);

  velocity_planner_ptr->set_steering_angle(velocity_planner_ptr->way_points->operator[](0));
  velocity_planner_ptr->set_steering_angle(velocity_planner_ptr->way_points->operator[](1));
  velocity_planner_ptr->set_steering_angle(velocity_planner_ptr->way_points->operator[](2));
  velocity_planner_ptr->set_steering_angle(velocity_planner_ptr->way_points->operator[](3));
  velocity_planner_ptr->set_steering_angle(velocity_planner_ptr->way_points->operator[](4));

  ASSERT_FLOAT_EQ(velocity_planner_ptr->way_points->at(1).point.front_wheel_angle_rad, 0.2477835F);
  ASSERT_FLOAT_EQ(velocity_planner_ptr->way_points->at(2).point.front_wheel_angle_rad, 0.514806F);
  ASSERT_FLOAT_EQ(velocity_planner_ptr->way_points->at(3).point.front_wheel_angle_rad, 0.2477835F);

  autoware::global_velocity_planner::State pose0, pose1, pose2, pose3;
  pose0.state.pose.position.x = 1;
  pose0.state.pose.position.y = 1;
  pose1.state.pose.position.x = 5;
  pose1.state.pose.position.y = 5;
  pose2.state.pose.position.x = 9;
  pose2.state.pose.position.y = 5;
  pose3.state.pose.position.x = 10;
  pose3.state.pose.position.y = 9;

  ASSERT_EQ(velocity_planner_ptr->get_closest_index(pose0), static_cast<size_t>(0));
  ASSERT_EQ(velocity_planner_ptr->get_closest_index(pose1), static_cast<size_t>(1));
  ASSERT_EQ(velocity_planner_ptr->get_closest_index(pose2), static_cast<size_t>(2));
  ASSERT_EQ(velocity_planner_ptr->get_closest_index(pose3), static_cast<size_t>(3));
}
TEST_F(GlobalVelocityPlannerTest, route_test)
{
  // create map
  const auto lane_id = lanelet::utils::getId();
  constexpr float64_t velocity_mps = 1.0;
  constexpr size_t n_points = 5;
  const auto lanelet_map_ptr = getALaneletMapWithLaneId(lane_id, velocity_mps, n_points);

  // create route message
  const auto had_map_route = getARoute(lane_id, 5.0F);
  TrajectoryPoint start;
  State pose;
  pose.state.pose = had_map_route.start_pose;

  velocity_planner_ptr->set_route(had_map_route, lanelet_map_ptr);
  velocity_planner_ptr->calculate_waypoints();
  velocity_planner_ptr->calculate_trajectory(pose);
  ASSERT_FALSE(velocity_planner_ptr->is_route_over());

  ASSERT_FALSE(velocity_planner_ptr->trajectory.points.empty());
  TrajectoryPoint p1;
  p1.pose.position.x = 0;
  p1.pose.position.y = 0;
  p1.longitudinal_velocity_mps = 1;
  TrajectoryPoint p2;
  p2.pose.position.x = 1;
  p2.pose.position.y = 0;
  p2.longitudinal_velocity_mps = 2;
  ASSERT_FLOAT_EQ(autoware::global_velocity_planner::find_velocity(p1, p2, 0.1F), 1.0954452F);

  ASSERT_FLOAT_EQ(autoware::global_velocity_planner::find_acceleration(p1, p2), 1.5F);

  // return trajectory should not be empty
  ASSERT_FALSE(velocity_planner_ptr->is_route_empty());
  velocity_planner_ptr->clear_route();
  ASSERT_TRUE(velocity_planner_ptr->is_route_empty());
}
