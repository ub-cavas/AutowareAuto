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
#include <global_velocity_planner/global_velocity_planner_node.hpp>
#include <had_map_utils/had_map_conversion.hpp>
#include <memory>
#include <string>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include "gtest/gtest.h"

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

using motion::motion_common::Real;
using motion::motion_common::VehicleConfig;

using autoware_auto_mapping_msgs::srv::HADMapService;
using autoware_auto_planning_msgs::msg::HADMapRoute;
using autoware_auto_mapping_msgs::msg::HADMapSegment;
using autoware_auto_mapping_msgs::msg::MapPrimitive;
using autoware_auto_planning_msgs::msg::Trajectory;
using VehicleState = autoware_auto_vehicle_msgs::msg::VehicleKinematicState;
using namespace std::chrono_literals;

// returns a map with a lane has given number of points(n_points)
// length of the lane will be n_points meters in y direction
lanelet::LaneletMapPtr get_lanelet_map(
  const lanelet::Id & id, const float64_t velocity,
  const size_t n_points)
{
  lanelet::Points3d right_points, left_points, center_points;
  constexpr float64_t resolution = 0.5;
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

HADMapRoute get_route(const int64_t lane_id, const float32_t length)
{
  HADMapRoute had_map_route;
  had_map_route.start_pose.position.x = 0;
  had_map_route.start_pose.position.y = 0;
  had_map_route.goal_pose.position.x = 0;
  had_map_route.goal_pose.position.y = length;

  MapPrimitive primitive;
  primitive.id = lane_id;
  primitive.primitive_type = "lane";
  HADMapSegment segment;

  segment.preferred_primitive_id = primitive.id;
  had_map_route.segments.push_back(segment);
  had_map_route.segments.front().primitives.push_back(primitive);

  return had_map_route;
}

inline geometry_msgs::msg::TransformStamped getDummyTransform(const VehicleState & State)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.transform.translation.x = 0.0;
  transform_stamped.transform.translation.y = 0.0;
  transform_stamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();
  transform_stamped.transform.rotation.w = q.w();
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = State.header.frame_id;
  return transform_stamped;
}


class GlobalVelocityPlannerNodeTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    ASSERT_FALSE(rclcpp::ok());
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    m_fake_node = std::make_shared<rclcpp::Node>("fake_node");
    m_fake_map_service =
      m_fake_node->create_service<HADMapService>(
      "HAD_Map_Service",
      std::bind(&GlobalVelocityPlannerNodeTest::send_fake_map, this, _1, _2, _3));
    // Publisher/Subscribers
    state_pub = m_fake_node->create_publisher<VehicleState>(
      "vehicle_state", rclcpp::QoS{10});

    route_pub = m_fake_node->create_publisher<HADMapRoute>(
      "route",
      rclcpp::QoS{10});

    trajectory_sub = m_fake_node->create_subscription<Trajectory>(
      "trajectory", rclcpp::QoS{10},
      [this](const Trajectory::SharedPtr msg) {
        this->trajectory_out = msg; this->received_trajectory = true;
      });
    br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->m_fake_node);

    rclcpp::NodeOptions node_options{};
    node_options.append_parameter_override("vehicle.cg_to_front_m", 1.0F);
    node_options.append_parameter_override("vehicle.cg_to_rear_m", 1.0F);
    node_options.append_parameter_override("vehicle.front_corner_stiffness", 0.1F);
    node_options.append_parameter_override("vehicle.rear_corner_stiffness", 0.1F);
    node_options.append_parameter_override("vehicle.mass_kg", 1500.0F);
    node_options.append_parameter_override("vehicle.yaw_inertia_kgm2", 12.0F);
    node_options.append_parameter_override("vehicle.width_m", 2.0F);
    node_options.append_parameter_override("vehicle.front_overhang_m", 0.5F);
    node_options.append_parameter_override("vehicle.rear_overhang_m", 0.5F);

    node_options.append_parameter_override("global_velocity_planner.trajectory_resolution", 0.5F);
    node_options.append_parameter_override("global_velocity_planner.lateral_acceleration", 2.0F);
    node_options.append_parameter_override(
      "global_velocity_planner.longitudinal_acceleration",
      2.0F);

    m_planner_ptr = std::make_shared<autoware::global_velocity_planner::GlobalVelocityPlannerNode>(
      node_options);
    m_lane_id = lanelet::utils::getId();
  }
  void send_fake_map(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<HADMapService::Request> request,
    const std::shared_ptr<HADMapService::Response> response)
  {
    (void)request_header;
    (void)request;
    autoware_auto_mapping_msgs::msg::HADMapBin map_msg;
    const auto map_ptr = get_lanelet_map(m_lane_id, 1, 50);
    autoware::common::had_map_utils::toBinaryMsg(map_ptr, map_msg);
    response->map = map_msg;
  }
  Trajectory::SharedPtr trajectory_out;
  autoware::common::types::bool8_t received_trajectory = false;
  std::shared_ptr<autoware::global_velocity_planner::GlobalVelocityPlannerNode> m_planner_ptr;
  rclcpp::Node::SharedPtr m_fake_node{nullptr};
  rclcpp::Service<HADMapService>::SharedPtr m_fake_map_service;
  rclcpp::Publisher<VehicleState>::SharedPtr state_pub;
  rclcpp::Publisher<HADMapRoute>::SharedPtr route_pub;
  rclcpp::Subscription<Trajectory>::SharedPtr trajectory_sub;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br;
  lanelet::Id m_lane_id;
};

TEST_F(GlobalVelocityPlannerNodeTest, check_trajectory) {
  const auto dt {50ms};
  const auto max_wait_time{std::chrono::seconds{1LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(m_fake_node);
  executor.add_node(m_planner_ptr);
  while (!received_trajectory) {
    VehicleState state;
    state.header.stamp = m_fake_node->now();
    state.header.frame_id = "base_link";
    state.state.pose.position.x = 0;
    state.state.pose.position.y = 0;
    state.state.pose.position.z = 0;
    state.state.pose.orientation.x = 0;
    state.state.pose.orientation.y = 0;
    state.state.pose.orientation.z = 0;
    geometry_msgs::msg::TransformStamped transform = getDummyTransform(state);
    transform.header.stamp = m_fake_node->now();
    br->sendTransform(transform);
    state_pub->publish(state);
    route_pub->publish(get_route(m_lane_id, 50));
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    executor.spin_once();
  }
  ASSERT_TRUE(received_trajectory);
  rclcpp::shutdown();
}
