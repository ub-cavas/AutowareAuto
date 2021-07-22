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

#include "tracking_nodes/multi_object_tracker_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace autoware
{
namespace tracking_nodes
{

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::perception::tracking::MultiObjectTracker;
using autoware::perception::tracking::MultiObjectTrackerOptions;
using autoware::perception::tracking::TrackerUpdateResult;
using autoware::perception::tracking::TrackerUpdateStatus;
using autoware_auto_msgs::msg::DetectedObjects;
using autoware_auto_msgs::msg::TrackedObjects;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
MultiObjectTracker init_tracker(rclcpp::Node & node)
{
  const float32_t max_distance =
    static_cast<float32_t>(node.declare_parameter(
      "association_max_distance").get<float64_t>());
  const float32_t max_area_ratio =
    static_cast<float32_t>(node.declare_parameter(
      "association_max_area_ratio").get<float64_t>());
  const bool consider_edge_for_big_detections = node.declare_parameter(
    "association_consider_edge_for_big_detection").get<bool>();
  const auto creation_policy = perception::tracking::TrackCreationPolicy::LidarClusterOnly;
  const float32_t default_variance =
    static_cast<float32_t>(node.declare_parameter(
      "ekf_default_variance").get<float64_t>());
  const float32_t noise_variance =
    static_cast<float32_t>(node.declare_parameter(
      "ekf_noise_variance").get<float64_t>());
  const std::chrono::nanoseconds pruning_time_threshold =
    std::chrono::milliseconds(
    node.declare_parameter(
      "pruning_time_threshold_ms").get<int64_t>());
  const std::size_t pruning_ticks_threshold =
    static_cast<std::size_t>(node.declare_parameter(
      "pruning_ticks_threshold").get<int64_t>());
  const std::string frame = node.declare_parameter("track_frame_id", "odom");

  MultiObjectTrackerOptions options{
    {max_distance, max_area_ratio, consider_edge_for_big_detections},
    {creation_policy, default_variance, noise_variance},
    pruning_time_threshold, pruning_ticks_threshold, frame};
  return MultiObjectTracker{options};
}

std::string status_to_string(TrackerUpdateStatus status)
{
  // Use a switch statement without default since it warns when not all cases are handled.
  switch (status) {
    case TrackerUpdateStatus::Ok: return "Ok";
    case TrackerUpdateStatus::WentBackInTime: return "WentBackInTime";
    case TrackerUpdateStatus::DetectionFrameMismatch: return "DetectionFrameMismatch";
    case TrackerUpdateStatus::TrackerFrameMismatch: return "TrackerFrameMismatch";
    case TrackerUpdateStatus::FrameNotGravityAligned: return "FrameNotGravityAligned";
    case TrackerUpdateStatus::InvalidShape: return "InvalidShape";
  }
  return "Invalid status";
}

}  // namespace

MultiObjectTrackerNode::MultiObjectTrackerNode(const rclcpp::NodeOptions & options)
:  Node("multi_object_tracker_node", options),
  m_tracker(init_tracker(*this)),
  m_history_depth(static_cast<size_t>(this->declare_parameter("history_depth", 20))),
  m_use_ndt(this->declare_parameter("use_ndt", true)),
  m_lidar_clusters_sub(create_subscription<autoware_auto_msgs::msg::DetectedObjects>(
      "detected_objects", rclcpp::QoS(m_history_depth), [this]
        (autoware_auto_msgs::msg::DetectedObjects::ConstSharedPtr msg) {
        process_lidar_clusters(msg);
      })),
  m_pub(create_publisher<TrackedObjects>("tracked_objects", m_history_depth))
{
  if (m_use_ndt) {
    m_odom_sub.subscribe(this, "odometry", rclcpp::QoS(m_history_depth).get_rmw_qos_profile());
    m_odom_cache_ptr = std::make_shared<OdomCache>(m_odom_sub, 20);
    m_odom_cache_ptr->setCacheSize(static_cast<uint32_t>(m_history_depth));
  } else {
    m_pose_sub.subscribe(this, "odometry", rclcpp::QoS(m_history_depth).get_rmw_qos_profile());
    m_pose_cache_ptr = std::make_shared<PoseCache>(m_pose_sub, 20);
    m_pose_cache_ptr->setCacheSize(static_cast<uint32_t>(m_history_depth));
  }
}

void MultiObjectTrackerNode::process_lidar_clusters(
  const autoware_auto_msgs::msg::DetectedObjects::ConstSharedPtr & msg)
{
  rclcpp::Time msg_stamp(msg->header.stamp.sec, msg->header.stamp.nanosec);
  const auto before = msg_stamp - std::chrono::milliseconds(30);
  const auto after = msg_stamp + std::chrono::milliseconds(30);

  if (m_use_ndt) {
    const auto matched_msgs = m_odom_cache_ptr->getInterval(before, after);
    if (matched_msgs.empty()) {
      RCLCPP_WARN(get_logger(), "No matching odom msg received for obj msg");
      return;
    }
    auto odom_msg = std::min_element(
      matched_msgs.begin(), matched_msgs.end(), [&msg](const auto &
      a, const auto & b) {
        rclcpp::Time t1(a->header.stamp);
        rclcpp::Time t2(b->header.stamp);
        rclcpp::Time msg_t(msg->header.stamp);
        return std::abs((t1 - msg_t).nanoseconds()) < std::abs((t2 - msg_t).nanoseconds());
      });
    process_using_odom(msg, *odom_msg);
  } else {
    const auto matched_msgs = m_pose_cache_ptr->getInterval(before, after);
    if (matched_msgs.empty()) {
      RCLCPP_WARN(get_logger(), "No matching pose msg received for obj msg");
      return;
    }
    auto pose_msg = std::min_element(
        matched_msgs.begin(), matched_msgs.end(), [&msg](const auto &
        a, const auto & b) {
          rclcpp::Time t1(a->header.stamp);
          rclcpp::Time t2(b->header.stamp);
          rclcpp::Time msg_t(msg->header.stamp);
          return std::abs((t1 - msg_t).nanoseconds()) < std::abs((t2 - msg_t).nanoseconds());
        });
    process_using_pose(msg, *pose_msg);
  }
}

void MultiObjectTrackerNode::process_using_odom(
  const DetectedObjects::ConstSharedPtr & objs,
  const Odometry::ConstSharedPtr & odom)
{
  TrackerUpdateResult result = m_tracker.update(*objs, *odom);
  if (result.status == TrackerUpdateStatus::Ok) {
    // The tracker returns its result in a unique_ptr, so the more efficient publish(unique_ptr<T>)
    // overload can be used.
    m_pub->publish(std::move(result.objects));
  } else {
    RCLCPP_WARN(
      get_logger(), "Tracker update for detection at time %d.%d failed. Reason: %s",
      objs->header.stamp.sec, objs->header.stamp.nanosec,
      status_to_string(result.status).c_str());
  }
}

void MultiObjectTrackerNode::process_using_pose(
  const autoware_auto_msgs::msg::DetectedObjects::ConstSharedPtr & objs,
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose)
{
  // Convert pose to odom because that is what the tracker wants
  nav_msgs::msg::Odometry::SharedPtr odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  odom_msg->header = pose->header;
  odom_msg->child_frame_id = "base_link";
  odom_msg->pose = pose->pose;
  process_using_odom(objs, odom_msg);
}

}  // namespace tracking_nodes
}  // namespace autoware

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tracking_nodes::MultiObjectTrackerNode)
