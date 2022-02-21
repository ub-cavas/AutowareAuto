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

// Scene modules
#include <scene_module/blind_spot/manager.hpp>
#include <scene_module/crosswalk/manager.hpp>
#include <scene_module/detection_area/manager.hpp>
#include <scene_module/intersection/manager.hpp>
#include <scene_module/stop_line/manager.hpp>
#include <behavior_velocity_planner_nodes/behavior_velocity_planner_node.hpp>

#include <lanelet2_routing/RoutingGraphContainer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_utils/autoware_utils.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <utilization/path_utilization.hpp>
#include <geometry/common_2d.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <time_utils/time_utils.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/order_movement.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <limits>
#include <algorithm>
#include <string>

namespace autoware
{
namespace planning
{
namespace behavior_velocity_planner_nodes
{
const std::uint32_t QOS_HISTORY_DEPTH = 1;
using namespace std::chrono_literals;

BehaviorVelocityPlannerNode::BehaviorVelocityPlannerNode(const rclcpp::NodeOptions & options)
: Node("behavior_velocity_planner_node", options),
  tf_buffer_ptr_{std::make_shared<tf2_ros::Buffer>(this->get_clock())},
  tf_listener_ptr_{std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_)},
  pub_trajectory_{this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
      "~/output/trajectory", rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)))},
  pub_diagnostic_status_{this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
      "~/output/stop_reason", rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)))},
  pub_markers_debug_{this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/debug/path", rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)))},
  sub_predicted_objects_{
    this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
      "~/input/dynamic_objects",
      rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)),
      std::bind(
        &BehaviorVelocityPlannerNode::callback_predicted_objects, this, std::placeholders::_1))},
  sub_cloud_no_ground_{this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "~/input/no_ground_pointcloud",
      rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)),
      std::bind(
        &BehaviorVelocityPlannerNode::callback_cloud_no_ground, this, std::placeholders::_1))},
  sub_vehicle_state_{
    this->create_subscription<autoware_auto_vehicle_msgs::msg::VehicleKinematicState>(
      "~/input/vehicle_velocity",
      rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)),
      std::bind(
        &BehaviorVelocityPlannerNode::callback_vehicle_state, this, std::placeholders::_1))},
  sub_path_with_lane_id_{
    this->create_subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>(
      "~/input/path_with_lane_id",
      rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)),
      std::bind(
        &BehaviorVelocityPlannerNode::callback_path_with_lane_id, this, std::placeholders::_1))},
  sub_order_movement_crosswalk_{
    this->create_subscription<autoware_auto_planning_msgs::msg::OrderMovement>(
      "~/input/external_crosswalk_states",
      rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)),
      std::bind(
        &BehaviorVelocityPlannerNode::callback_order_movement_crosswalk,
        this,
        std::placeholders::_1))},
  sub_order_movement_intersection_{
    this->create_subscription<autoware_auto_planning_msgs::msg::OrderMovement>(
      "~/input/external_intersection_states",
      rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)),
      std::bind(
        &BehaviorVelocityPlannerNode::callback_order_movement_intersection,
        this,
        std::placeholders::_1))},
  map_client_{
    this->create_client<autoware_auto_mapping_msgs::srv::HADMapService>("HAD_Map_Client")},
  planner_data_(*this)
{
  using std::chrono_literals::operator""ms;

  // Parameters
  forward_path_length_ = this->declare_parameter("forward_path_length", 1000.0);
  backward_path_length_ = this->declare_parameter("backward_path_length", 5.0);
  // TODO(yukkysaito): This will become unnecessary when acc output from localization is available.
  planner_data_.accel_lowpass_gain_ = this->declare_parameter("lowpass_gain", 0.5);

  // Initialize PlannerManager
  if (this->declare_parameter("launch_stop_line", true)) {
    planner_manager_.launchSceneModule(std::make_shared<StopLineModuleManager>(*this));
  }
  if (this->declare_parameter("launch_crosswalk", true)) {
    planner_manager_.launchSceneModule(std::make_shared<CrosswalkModuleManager>(*this));
  }
  if (this->declare_parameter("launch_intersection", true)) {
    planner_manager_.launchSceneModule(std::make_shared<IntersectionModuleManager>(*this));
  }
  if (this->declare_parameter("launch_blind_spot", true)) {
    planner_manager_.launchSceneModule(std::make_shared<BlindSpotModuleManager>(*this));
  }
  if (this->declare_parameter("launch_detection_area", true)) {
    planner_manager_.launchSceneModule(std::make_shared<DetectionAreaModuleManager>(*this));
  }

  // Request binary map from the map loader node
  this->request_osm_binary_map();
}

void BehaviorVelocityPlannerNode::request_osm_binary_map()
{
  while (rclcpp::ok() && !map_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "HAD map service not available yet. Waiting...");
  }
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(
      this->get_logger(), "Client interrupted while waiting for map service to appear. Exiting.");
  }

  auto request = std::make_shared<autoware_auto_mapping_msgs::srv::HADMapService_Request>();
  request->requested_primitives.push_back(
    autoware_auto_mapping_msgs::srv::HADMapService_Request::FULL_MAP);

  auto result = map_client_->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Service call failed");
    throw std::runtime_error("Lanelet2GlobalPlannerNode: Map service call fail");
  }

  // copy message to map
  autoware_auto_mapping_msgs::msg::HADMapBin msg = result.get()->map;

  processs_had_map_bin_lanelet(msg);
}

void BehaviorVelocityPlannerNode::processs_had_map_bin_lanelet(
  const autoware_auto_mapping_msgs::msg::HADMapBin & had_map_bin)
{
  // Load map
  planner_data_.lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    had_map_bin,
    planner_data_.lanelet_map,
    &planner_data_.traffic_rules,
    &planner_data_.routing_graph);

  // Build graph
  {
    using lanelet::Locations;
    using lanelet::Participants;
    using lanelet::routing::RoutingGraph;
    using lanelet::routing::RoutingGraphConstPtr;
    using lanelet::routing::RoutingGraphContainer;
    using lanelet::traffic_rules::TrafficRulesFactory;

    const auto traffic_rules =
      TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
    const auto pedestrian_rules =
      TrafficRulesFactory::create(Locations::Germany, Participants::Pedestrian);

    RoutingGraphConstPtr vehicle_graph =
      RoutingGraph::build(*planner_data_.lanelet_map, *traffic_rules);
    RoutingGraphConstPtr pedestrian_graph =
      RoutingGraph::build(*planner_data_.lanelet_map, *pedestrian_rules);
    RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});

    planner_data_.overall_graphs = std::make_shared<const RoutingGraphContainer>(overall_graphs);
  }
}

void BehaviorVelocityPlannerNode::callback_predicted_objects(
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg_in)
{
  planner_data_.dynamic_objects = msg_in;
}

void BehaviorVelocityPlannerNode::callback_cloud_no_ground(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_in)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_ptr_->lookupTransform(
      "map", msg_in->header.frame_id, msg_in->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(get_logger(), "no transform found for no_ground_pointcloud: %s", e.what());
    return;
  }

  using CloudView = point_cloud_msg_wrapper::PointCloud2View<common::types::PointXYZI>;
  CloudView cloud_view(*msg_in);

  using CloudModifier = point_cloud_msg_wrapper::PointCloud2Modifier<common::types::PointXYZI>;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_ptr_trans =
    std::make_shared<sensor_msgs::msg::PointCloud2>();

  Eigen::Affine3f affine = tf2::transformToEigen(transform.transform).cast<float_t>();
  CloudModifier cloud_modifier_trans(*cloud_ptr_trans, "map");
  cloud_modifier_trans.resize(static_cast<uint32_t>(cloud_view.size()));
  std::transform(
    cloud_view.cbegin(),
    cloud_view.cend(),
    cloud_modifier_trans.begin(),
    [&affine](const common::types::PointXYZI & point) {
      Eigen::Vector4f vec_point(point.x, point.y, point.z, 1.0f);
      Eigen::Vector4f vec_point_trans(affine.matrix() * vec_point);
      return common::types::PointXYZI{
        vec_point_trans.x(), vec_point_trans.y(), vec_point_trans.z(), point.intensity};
    });

  planner_data_.no_ground_pointcloud = cloud_ptr_trans;
}

void BehaviorVelocityPlannerNode::callback_vehicle_state(
  const autoware_auto_vehicle_msgs::msg::VehicleKinematicState::ConstSharedPtr msg_in)
{
  std::shared_ptr<geometry_msgs::msg::TwistStamped> twist_stamped{
    std::make_shared<geometry_msgs::msg::TwistStamped>()};
  twist_stamped->header.stamp = msg_in->header.stamp;
  twist_stamped->twist.linear.x = msg_in->state.longitudinal_velocity_mps;
  twist_stamped->twist.linear.y = msg_in->state.lateral_velocity_mps;
  twist_stamped->twist.linear.z = 0.0f;

  planner_data_.current_velocity = twist_stamped;
  planner_data_.updateCurrentAcc();

  // Add velocity to buffer
  planner_data_.velocity_buffer.push_front(*twist_stamped);
  const auto now = this->now();

  while (true) {
    // Check oldest data time
    const auto time_diff = now - planner_data_.velocity_buffer.back().header.stamp;

    // Finish when oldest data is newer than threshold
    if (time_diff.seconds() <= PlannerData::velocity_buffer_time_sec) {
      break;
    }

    // Remove old data
    planner_data_.velocity_buffer.pop_back();
  }
}

void BehaviorVelocityPlannerNode::callback_path_with_lane_id(
  const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg_in)
{
  auto transform2pose = [](const geometry_msgs::msg::TransformStamped & transform) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      return pose;
    };

  // Check ready
  try {
    planner_data_.current_pose =
      transform2pose(tf_buffer_ptr_->lookupTransform("map", "base_link", tf2::TimePointZero));
  } catch (tf2::TransformException & e) {
    RCLCPP_INFO(get_logger(), "waiting for transform from `map` to `base_link`");
    return;
  }

  if (!is_data_ready()) {
    return;
  }

  // Plan path velocity
  const auto velocity_planned_path =
    planner_manager_.planPathVelocity(std::make_shared<const PlannerData>(planner_data_), *msg_in);

  auto to_path = [](const autoware_auto_planning_msgs::msg::PathWithLaneId & path_with_id) {
      autoware_auto_planning_msgs::msg::Path path;
      for (const auto & path_point : path_with_id.points) {
        path.points.push_back(path_point.point);
      }
      return path;
    };

  // screening
  const auto filtered_path = filterLitterPathPoint(to_path(velocity_planned_path));

  // interpolation
  const auto interpolated_path_msg =
    interpolatePath(filtered_path, forward_path_length_, this->get_logger());

  // check stop point
  auto output_path_msg = filterStopPathPoint(interpolated_path_msg);
  output_path_msg.header.frame_id = "map";
  output_path_msg.header.stamp = this->now();

  // TODO(someone): This must be updated in each scene module, but copy from input message for now.
  output_path_msg.drivable_area = msg_in->drivable_area;

  if (output_path_msg.points.size() > 2) {
    output_path_msg.points.erase(
      output_path_msg.points.begin(), output_path_msg.points.begin() + 2);
  }

  pub_trajectory_->publish(path_to_trajectory(output_path_msg));
  pub_diagnostic_status_->publish(planner_manager_.getStopReasonDiag());

  if (pub_markers_debug_->get_subscription_count() > 0) {
    publish_debug_marker(output_path_msg);
  }
}

void BehaviorVelocityPlannerNode::callback_order_movement_crosswalk(
  const autoware_auto_planning_msgs::msg::OrderMovement::ConstSharedPtr msg_in)
{
  planner_data_.external_crosswalk_status_input = *msg_in;
}

void BehaviorVelocityPlannerNode::callback_order_movement_intersection(
  const autoware_auto_planning_msgs::msg::OrderMovement::ConstSharedPtr msg_in)
{
  planner_data_.external_intersection_status_input = *msg_in;
}

bool BehaviorVelocityPlannerNode::is_data_ready()
{
  const auto & d = planner_data_;

  // from tf
  if (d.current_pose.header.frame_id == "") {
    return false;
  }
  // from callbacks
  if (!d.current_velocity) {
    return false;
  }
  //  if (!d.dynamic_objects) {
  //    return false;
  //  }
  //  if (!d.no_ground_pointcloud) {
  //    return false;
  //  }
  if (!d.lanelet_map) {
    return false;
  }
  return true;
}

autoware_auto_planning_msgs::msg::Trajectory BehaviorVelocityPlannerNode::path_to_trajectory(
  const autoware_auto_planning_msgs::msg::Path & path)
{
  autoware_auto_planning_msgs::msg::Trajectory trajectory_output;

  if (path.points.empty()) {
    std::cout << "Path is empty!" << std::endl;
    return trajectory_output;
  }

  size_t trajectory_length = std::min(
    path.points.size(),
    static_cast<size_t>(autoware_auto_planning_msgs::msg::Trajectory::CAPACITY));

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> trajectory_points;
  trajectory_points.resize(trajectory_length);

  // fill trajectory points with pose and velocity
  for (size_t i = 0; i < trajectory_length; ++i) {
    auto const & path_point = path.points.at(i);
    auto & trajectory_point = trajectory_points.at(i);

    trajectory_point.pose = path_point.pose;
    trajectory_point.longitudinal_velocity_mps = static_cast<float>(path_point.twist.linear.x);
    trajectory_point.lateral_velocity_mps = static_cast<float>(path_point.twist.linear.y);
  }

  // calculate missing fields in trajectory
  set_steering_angle(trajectory_points);

  set_time_from_start(trajectory_points);

  // create trajectory message
  trajectory_output.header = path.header;
  trajectory_output.points.resize(trajectory_length);
  for (size_t i = 0; i < trajectory_length; i++) {
    trajectory_output.points.at(i) = trajectory_points.at(i);
  }

  return trajectory_output;
}

void BehaviorVelocityPlannerNode::set_steering_angle(
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & trajectory_points)
{
  // set steering angle
  const auto wheel_base = static_cast<float>(planner_data_.vehicle_constants_.wheel_base);
  for (size_t i = 1; i < trajectory_points.size() - 1; i++) {
    auto & pt = trajectory_points.at(i);
    const auto & prev_pt = trajectory_points.at(i - 1);
    const auto & next_pt = trajectory_points.at(i + 1);
    const auto curvature =
      calculate_curvature(prev_pt.pose.position, pt.pose.position, next_pt.pose.position);
    pt.front_wheel_angle_rad = std::atan(wheel_base * curvature);
  }
}

float BehaviorVelocityPlannerNode::distance2d(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  const auto diff = autoware::common::geometry::minus_2d(p1, p2);
  return autoware::common::geometry::norm_2d(diff);
}

float BehaviorVelocityPlannerNode::calculate_curvature(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3)
{
  const float epsilon = std::numeric_limits<float>::epsilon();
  float den = std::max(distance2d(p1, p2) * distance2d(p2, p3) * distance2d(p3, p1), epsilon);
  const float curvature =
    2.0F * static_cast<float>((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / den;
  return curvature;
}

void BehaviorVelocityPlannerNode::set_time_from_start(
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & trajectory_points)
{
  // set time_from_start
  double accumulated_time = 0;
  trajectory_points.at(0).time_from_start.sec = 0;
  trajectory_points.at(0).time_from_start.nanosec = 0;
  for (size_t i = 1; i < trajectory_points.size(); i++) {
    auto & pt = trajectory_points.at(i);
    const auto & prev_pt = trajectory_points.at(i - 1);
    const double distance_x = pt.pose.position.x - prev_pt.pose.position.x;
    const double distance_y = pt.pose.position.y - prev_pt.pose.position.y;
    const double distance = std::sqrt(distance_x * distance_x + distance_y * distance_y);
    const double velocity = prev_pt.longitudinal_velocity_mps;
    accumulated_time += distance / std::max(velocity, 0.5);
    std::chrono::nanoseconds duration(static_cast<int64_t>(accumulated_time * 1e9));
    pt.time_from_start = time_utils::to_message(duration);
  }
}

void BehaviorVelocityPlannerNode::publish_debug_marker(
  const autoware_auto_planning_msgs::msg::Path & path)
{
  visualization_msgs::msg::MarkerArray output_msg;
  for (size_t i = 0; i < path.points.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.id = static_cast<int32_t>(i);
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.pose = path.points.at(i).pose;
    marker.scale.y = marker.scale.z = 0.05;
    marker.scale.x = 0.25;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.color.a = 0.999f;  // Don't forget to set the alpha!
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    output_msg.markers.push_back(marker);
  }
  pub_markers_debug_->publish(output_msg);
}

autoware_auto_planning_msgs::msg::Path BehaviorVelocityPlannerNode::transform_path(
  const autoware_auto_planning_msgs::msg::Path & path,
  const std::string & source_frame_id,
  const std::string & target_frame_id)
{
  autoware_auto_planning_msgs::msg::Path transformed_path;
  transformed_path.header = path.header;
  transformed_path.drivable_area = path.drivable_area;

  geometry_msgs::msg::TransformStamped transform_data;
  // Check ready
  try {
    transform_data =
      tf_buffer_ptr_->lookupTransform(target_frame_id, source_frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException & e) {
    RCLCPP_INFO(
      get_logger(), "waiting for transform from " + source_frame_id + " to " + target_frame_id);
    return transformed_path;
  }

  auto pose_to_matrix = [](const geometry_msgs::msg::Pose & pose) {
      Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
      const auto & pos = pose.position;
      const auto & ori = pose.orientation;
      Eigen::Quaterniond quat(ori.w, ori.x, ori.y, ori.z);
      mat.topLeftCorner<3, 3>() = quat.toRotationMatrix();
      mat.topRightCorner<3, 1>() = Eigen::Vector3d(pos.x, pos.y, pos.z);
      return mat;
    };

  auto transform_stamped_to_matrix =
    [&pose_to_matrix](const geometry_msgs::msg::TransformStamped & transform_stamped) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = transform_stamped.transform.translation.x;
      pose.position.y = transform_stamped.transform.translation.y;
      pose.position.z = transform_stamped.transform.translation.z;
      pose.orientation = transform_stamped.transform.rotation;

      return pose_to_matrix(pose);
    };

  auto transform_pose = [&pose_to_matrix, &transform_stamped_to_matrix, transform_data](
    const geometry_msgs::msg::Pose & pose) {
      geometry_msgs::msg::Pose pose_trans;
      auto mat_pose = pose_to_matrix(pose);
      auto mat_transform = transform_stamped_to_matrix(transform_data);
      Eigen::Matrix4d mat_transformed = mat_transform * mat_pose;
      Eigen::Quaterniond quat(mat_transformed.topLeftCorner<3, 3>());
      const Eigen::Vector3d & trans = mat_transformed.topRightCorner<3, 1>();
      pose_trans.position.x = trans.x();
      pose_trans.position.y = trans.y();
      pose_trans.position.z = trans.z();
      pose_trans.orientation.x = quat.x();
      pose_trans.orientation.y = quat.y();
      pose_trans.orientation.z = quat.z();
      pose_trans.orientation.w = quat.w();
      return pose_trans;
    };

  transformed_path.points.resize(path.points.size());
  for (size_t i = 0; i < path.points.size(); ++i) {
    transformed_path.points.at(i).pose = transform_pose(path.points.at(i).pose);
    transformed_path.points.at(i).twist = path.points.at(i).twist;
    transformed_path.points.at(i).is_final = path.points.at(i).is_final;
  }
  return transformed_path;
}


}  // namespace behavior_velocity_planner_nodes
}  // namespace planning
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::planning::behavior_velocity_planner_nodes::BehaviorVelocityPlannerNode)
