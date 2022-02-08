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

#include <common/types.hpp>
#include <geometry/common_2d.hpp>
#include <had_map_utils/had_map_conversion.hpp>
#include <motion_common/config.hpp>

#include <memory>

#include "global_velocity_planner/global_velocity_planner_node.hpp"

namespace autoware
{
namespace global_velocity_planner
{

GlobalVelocityPlannerNode::GlobalVelocityPlannerNode(const rclcpp::NodeOptions & options)
: Node("global_velocity_planner", options)
{
  init();
}

void GlobalVelocityPlannerNode::init()
{
  using rclcpp::QoS;
  using namespace std::chrono_literals;
  const VehicleConfig vehicle_param{
    static_cast<Real>(declare_parameter("vehicle.cg_to_front_m").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.cg_to_rear_m").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.front_corner_stiffness").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.rear_corner_stiffness").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.mass_kg").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.yaw_inertia_kgm2").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.width_m").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.front_overhang_m").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.rear_overhang_m").get<float32_t>())};
  const global_velocity_planner::GlobalVelocityPlannerConfig planner_config{
    static_cast<float32_t>(
      declare_parameter("global_velocity_planner.trajectory_resolution").get<float32_t>()),
    static_cast<float32_t>(
      declare_parameter("global_velocity_planner.lateral_acceleration").get<float32_t>()),
    static_cast<float32_t>(
      declare_parameter("global_velocity_planner.longitudinal_acceleration").get<float32_t>())};

  velocity_planner = std::make_unique<GlobalVelocityPlanner>(vehicle_param, planner_config);

  // Setup Tf Buffer with listener
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(
    *m_tf_buffer, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false);
  m_route_sub = this->create_subscription<HADMapRoute>(
    "route", QoS{10}, [this](const HADMapRoute::SharedPtr msg) {
      route_callback(msg);
    });

  // Setup publishers
  global_trajectory_pub = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
    "trajectory", QoS{10});

  // Setup state subscriber
  state_sub = this->create_subscription<State>(
    "vehicle_state", QoS{10}, [this](const State::SharedPtr msg) {
      state_callback(msg);
    });

  m_map_client = this->create_client<HADMapService>("HAD_Map_Service");
  while (!m_map_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for service...");
  }
}

void GlobalVelocityPlannerNode::route_callback(const HADMapRoute::SharedPtr & msg)
{
  RCLCPP_INFO(get_logger(), "Received route");
  m_route = msg;
  auto request = std::make_shared<HADMapService::Request>();
  request->requested_primitives.push_back(HADMapService::Request::FULL_MAP);
  auto result = m_map_client->async_send_request(
    request, std::bind(&GlobalVelocityPlannerNode::map_response, this, std::placeholders::_1));
}

void GlobalVelocityPlannerNode::map_response(rclcpp::Client<HADMapService>::SharedFuture future)
{
  m_lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  autoware::common::had_map_utils::fromBinaryMsg(future.get()->map, m_lanelet_map_ptr);

  RCLCPP_INFO(get_logger(), "Received map");
  velocity_planner->clear_route();
  velocity_planner->set_route(*m_route, m_lanelet_map_ptr);
  velocity_planner->calculate_waypoints();
  RCLCPP_INFO(get_logger(), "Global path is created");
}

State GlobalVelocityPlannerNode::transform_to_map(const State & state)
{
  geometry_msgs::msg::TransformStamped tf;

  try {
    tf = m_tf_buffer->lookupTransform(
      "map", state.header.frame_id, time_utils::from_message(state.header.stamp));
  } catch (const tf2::ExtrapolationException &) {
    tf = m_tf_buffer->lookupTransform("map", state.header.frame_id, tf2::TimePointZero);
  }

  State transformed_state;
  motion::motion_common::doTransform(state, transformed_state, tf);
  transformed_state.header.frame_id = "map";
  transformed_state.header.stamp = state.header.stamp;
  return transformed_state;
}

void GlobalVelocityPlannerNode::state_callback(const State::SharedPtr & msg)
{
  // Do nothing if localization result is not received yet.
  if (!m_tf_buffer->canTransform("map", msg->header.frame_id, tf2::TimePointZero)) {
    RCLCPP_INFO(get_logger(), "Waiting for localization result to become available");
    return;
  }
  pose = transform_to_map(*msg);
  if (velocity_planner->is_route_ready && !velocity_planner->is_route_empty()) {
    if (velocity_planner->is_route_over()) {
      velocity_planner->is_route_ready = false;
      RCLCPP_INFO(get_logger(), "Route over");
      velocity_planner->clear_route();
      return;
    }
    velocity_planner->calculate_trajectory(pose);
    velocity_planner->trajectory.header.frame_id = "map";
    velocity_planner->trajectory.header.stamp = msg->header.stamp;
    global_trajectory_pub->publish(velocity_planner->trajectory);
  }
}
}  // namespace global_velocity_planner
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::global_velocity_planner::GlobalVelocityPlannerNode)
