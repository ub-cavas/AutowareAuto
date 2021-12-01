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

#include <tracking_nodes/multi_object_tracker_node.hpp>

#include <geometry/bounding_box/bounding_box_common.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <lidar_utils/cluster_utils/point_clusters_view.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <time_utils/time_utils.hpp>

#include <cstddef>
#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <utility>
#include <vector>


namespace autoware
{
namespace tracking_nodes
{

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::perception::tracking::MultiObjectTracker;
using autoware::perception::tracking::MultiObjectTrackerOptions;
using autoware::perception::tracking::DetectedObjectsUpdateResult;
using autoware::perception::tracking::TrackerUpdateStatus;
using autoware::perception::tracking::GreedyRoiAssociatorConfig;
using autoware::perception::tracking::CameraIntrinsics;
using autoware::perception::tracking::VisionPolicyConfig;
using autoware::perception::tracking::LidarOnlyPolicy;
using autoware::perception::tracking::LidarClusterIfVisionPolicy;
using autoware::perception::tracking::TrackCreator;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using autoware_auto_perception_msgs::msg::TrackedObjects;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
constexpr std::chrono::milliseconds kMaxLidarEgoStateStampDiff{100};
constexpr std::chrono::milliseconds kMaxVisionEgoStateStampDiff{100};
constexpr std::int64_t kDefaultHistoryDepth{20};
constexpr std::int64_t kDefaultPoseHistoryDepth{100};

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

// Convert pose msg to odom msg
Odometry::SharedPtr to_odom(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose_msg)
{
  nav_msgs::msg::Odometry::SharedPtr odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  odom_msg->header = pose_msg->header;
  odom_msg->child_frame_id = "base_link";
  odom_msg->pose = pose_msg->pose;
  return odom_msg;
}

// Get msg closest to the given timestamp from the list of messages
template<typename T>
T get_closest_match(const std::vector<T> & matched_msgs, const rclcpp::Time & stamp)
{
  return *std::min_element(
    matched_msgs.begin(), matched_msgs.end(), [&stamp](const auto &
    a, const auto & b) {
      const rclcpp::Time t1(a->header.stamp);
      const rclcpp::Time t2(b->header.stamp);
      return std::abs((t1 - stamp).nanoseconds()) < std::abs((t2 - stamp).nanoseconds());
    });
}

DetectedObjects convert_unassigned_clusters_to_detected_objects(
  const autoware_auto_perception_msgs::msg::PointClusters & clusters,
  const DetectedObjectsUpdateResult & update_result)
{
  using geometry_msgs::msg::Point32;
  using geometry_msgs::build;

  common::lidar_utils::PointClustersView clusters_msg_view{clusters};
  DetectedObjects detections_from_clusters;
  detections_from_clusters.header = clusters.header;
  detections_from_clusters.objects.reserve(clusters_msg_view.size());
  for (const auto idx : update_result.unassigned_clusters_indices) {
    if (idx >= clusters_msg_view.size()) {
      throw std::runtime_error("Wrong cluster idx");
    }
    autoware_auto_perception_msgs::msg::DetectedObject detected_object;
    detected_object.existence_probability = 1.0F;
    // Set shape as a convex hull of the cluster.
    auto cluster_view = clusters_msg_view[static_cast<std::uint32_t>(idx)];
    std::list<autoware_auto_perception_msgs::msg::PointClusters::_points_type::value_type>
    point_list{
      cluster_view.begin(), cluster_view.end()};
    const auto hull_end_iter = common::geometry::convex_hull(point_list);

    for (auto iter = point_list.begin(); iter != hull_end_iter; ++iter) {
      const auto & hull_point = *iter;
      detected_object.shape.polygon.points.push_back(
        build<Point32>().x(hull_point.x).y(hull_point.y).z(0.0F));
    }
    common::geometry::bounding_box::compute_height(
      cluster_view.begin(),
      cluster_view.end(),
      detected_object.shape);

    // Compute the centroid
    geometry_msgs::msg::Point32 sum;
    for (const auto & point : detected_object.shape.polygon.points) {
      sum = common::geometry::plus_2d(sum, point);
    }
    const auto centroid = common::geometry::times_2d(
      sum, 1.0F / static_cast<float>(detected_object.shape.polygon.points.size()));
    auto & detected_object_position = detected_object.kinematics.pose_with_covariance.pose.position;
    detected_object_position.x = static_cast<decltype(detected_object_position.x)>(centroid.x);
    detected_object_position.y = static_cast<decltype(detected_object_position.y)>(centroid.y);
    detected_object_position.z = static_cast<decltype(detected_object_position.z)>(centroid.z);
    for (auto & point : detected_object.shape.polygon.points) {
      // We assume here a zero orientation as we don't care about the orientation of the convex
      // hull. This then becomes a poor man's transformation into the object-local coordinates.
      point = common::geometry::minus_2d(point, centroid);
    }

    // Compute the classification
    autoware_auto_perception_msgs::msg::ObjectClassification label;
    label.classification = autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
    label.probability = 1.0F;
    detected_object.classification.emplace_back(label);

    detections_from_clusters.objects.push_back(detected_object);
  }
  return detections_from_clusters;
}

}  // namespace

MultiObjectTrackerNode::MultiObjectTrackerNode(const rclcpp::NodeOptions & options)
:  Node("multi_object_tracker_node", options),
  m_use_ndt{this->declare_parameter("use_ndt", true)},
  m_use_vision{this->declare_parameter("use_vision", true)},
  m_history_depth{static_cast<std::size_t>(
      declare_parameter("history_depth", kDefaultHistoryDepth))},
  m_tf_listener{m_tf_buffer},
  m_tracker{init_tracker(m_use_vision)},
  m_track_publisher{create_publisher<TrackedObjects>("tracked_objects", m_history_depth)},
  m_leftover_publisher{create_publisher<DetectedObjects>("leftover_clusters", m_history_depth)},
  m_visualize_track_creation{this->declare_parameter("visualize_track_creation", false)}
{
  const auto pose_history_depth =
    static_cast<size_t>(declare_parameter("pose_history_depth", kDefaultPoseHistoryDepth));
  m_odom_cache = std::make_unique<OdomCache>();
  m_odom_cache->setCacheSize(static_cast<std::uint32_t>(pose_history_depth));
  if (m_use_ndt) {
    m_odom_subscription = create_subscription<OdometryMsg>(
      "ego_state", rclcpp::QoS{pose_history_depth},
      std::bind(&MultiObjectTrackerNode::odometry_callback, this, std::placeholders::_1));
  } else {
    m_pose_subscription = create_subscription<PoseMsg>(
      "ego_state", rclcpp::QoS{pose_history_depth},
      std::bind(&MultiObjectTrackerNode::pose_callback, this, std::placeholders::_1));
  }

  const auto use_detected_objects = this->declare_parameter("use_detected_objects", true);
  const auto use_raw_clusters = this->declare_parameter("use_raw_clusters", true);

  if (use_raw_clusters && use_detected_objects) {
    std::runtime_error(
      "Cannot use raw clusters and detected objects interfaces at the same time for now.\n"
      "It will be possible later, but for now both inputs are generated from the clustering node.\n"
      "As of now, only one of those should be used.");
  }

  if (use_detected_objects) {
    m_detected_objects_subscription = create_subscription<DetectedObjects>(
      "detected_objects", rclcpp::QoS{m_history_depth},
      std::bind(&MultiObjectTrackerNode::detected_objects_callback, this, std::placeholders::_1));
  }

  if (use_raw_clusters) {
    m_clusters_subscription = create_subscription<ClustersMsg>(
      "clusters", rclcpp::QoS{m_history_depth},
      std::bind(&MultiObjectTrackerNode::clusters_callback, this, std::placeholders::_1));
  }

  // Initialize vision callbacks if vision is configured to be used:
  if (m_use_vision) {
    auto num_vision_topics = static_cast<size_t>(this->declare_parameter("num_vision_topics", 1));

    m_vision_subscriptions.resize(num_vision_topics);

    for (size_t i = 0U; i < m_vision_subscriptions.size(); ++i) {
      const auto topic_name = "classified_rois" + std::to_string(i + 1);
      m_vision_subscriptions[i] = create_subscription<ClassifiedRoiArray>(
        topic_name, rclcpp::QoS{m_history_depth},
        std::bind(
          &MultiObjectTrackerNode::classified_roi_callback, this,
          std::placeholders::_1));
    }
  }

  if (m_visualize_track_creation) {
    if (!m_use_vision) {
      throw std::runtime_error(
              "Visualization can only be enabled if the vision detections are enabled.");
    }
    m_track_creating_clusters_pub =
      create_publisher<DetectedObjects>("associated_detections", m_history_depth);
  }
}

MultiObjectTrackerNode::TrackerVariant MultiObjectTrackerNode::init_tracker(
  const common::types::bool8_t use_vision)
{
  const float32_t max_distance =
    static_cast<float32_t>(declare_parameter(
      "object_association.max_distance").get<float64_t>());
  const float32_t max_area_ratio =
    static_cast<float32_t>(declare_parameter(
      "object_association.max_area_ratio").get<float64_t>());
  const bool consider_edge_for_big_detections = declare_parameter(
    "object_association.consider_edge_for_big_detection").get<bool>();

  const auto default_variance = declare_parameter(
    "ekf_default_variance").get<float64_t>();
  const auto noise_variance = declare_parameter(
    "ekf_noise_variance").get<float64_t>();
  const std::chrono::nanoseconds pruning_time_threshold =
    std::chrono::milliseconds(
    declare_parameter(
      "pruning_time_threshold_ms").get<int64_t>());
  const std::size_t pruning_ticks_threshold =
    static_cast<std::size_t>(declare_parameter(
      "pruning_ticks_threshold").get<int64_t>());
  const std::string frame = declare_parameter("track_frame_id", "odom");

  GreedyRoiAssociatorConfig vision_config{};
  std::cerr << "use_vision: " << use_vision << std::endl;
  if (use_vision) {
    // There is no reason to have vision and use LidarClusterOnly policy. So, update the policy
    vision_config.intrinsics = {
      static_cast<std::size_t>(declare_parameter(
        "vision_association.intrinsics.width").get<int64_t>()),
      static_cast<std::size_t>(declare_parameter(
        "vision_association.intrinsics.height").get<int64_t>()),
      static_cast<float32_t>(declare_parameter(
        "vision_association.intrinsics.fx").get<float32_t>()),
      static_cast<float32_t>(declare_parameter(
        "vision_association.intrinsics.fy").get<float32_t>()),
      static_cast<float32_t>(declare_parameter(
        "vision_association.intrinsics.ox").get<float32_t>()),
      static_cast<float32_t>(declare_parameter(
        "vision_association.intrinsics.oy").get<float32_t>()),
      static_cast<float32_t>(declare_parameter(
        "vision_association.intrinsics.skew").get<float32_t>())
    };


    vision_config.iou_threshold = static_cast<float32_t>(declare_parameter(
        "vision_association.iou_threshold").get<float32_t>());

    VisionPolicyConfig vision_policy_cfg;
    vision_policy_cfg.associator_cfg = vision_config;
    vision_policy_cfg.max_vision_lidar_timestamp_diff = std::chrono::milliseconds(
      declare_parameter(
        "vision_association.timestamp_diff_ms").get<int64_t>());
    auto track_creation_policy = std::make_shared<LidarClusterIfVisionPolicy>(
      vision_policy_cfg, default_variance, noise_variance, m_tf_buffer);
    TrackCreator<LidarClusterIfVisionPolicy> track_creator{track_creation_policy};
    MultiObjectTrackerOptions options{
      {max_distance, max_area_ratio, consider_edge_for_big_detections}, vision_config,
      pruning_time_threshold, pruning_ticks_threshold, frame};
    return TrackerVariant{Tracker<LidarClusterIfVisionPolicy>{options, track_creator, m_tf_buffer}};
  }
  auto track_creation_policy =
    std::make_shared<LidarOnlyPolicy>(default_variance, noise_variance, m_tf_buffer);
  TrackCreator<LidarOnlyPolicy> track_creator{track_creation_policy};
  MultiObjectTrackerOptions options{
    {max_distance, max_area_ratio, consider_edge_for_big_detections}, vision_config,
    pruning_time_threshold, pruning_ticks_threshold, frame};
  return TrackerVariant{Tracker<LidarOnlyPolicy>{options, track_creator, m_tf_buffer}};
}

void MultiObjectTrackerNode::clusters_callback(const ClustersMsg::ConstSharedPtr objs)
{
  const rclcpp::Time msg_stamp{objs->header.stamp.sec, objs->header.stamp.nanosec};
  const auto earliest_time = msg_stamp - kMaxLidarEgoStateStampDiff;
  const auto latest_time = msg_stamp + kMaxLidarEgoStateStampDiff;
  const auto matched_msgs = m_odom_cache->getInterval(earliest_time, latest_time);
  if (matched_msgs.empty()) {
    RCLCPP_WARN(get_logger(), "No matching odom msgs received for clusters msg");
    return;
  }

  DetectedObjectsUpdateResult result;
  mpark::visit(
    [&objs, &matched_msgs, &result](auto & tracker) {
      result = tracker.update(*objs, *get_closest_match(matched_msgs, objs->header.stamp));
    }, m_tracker);

  if (result.status == TrackerUpdateStatus::Ok) {
    m_track_publisher->publish(result.tracks);
    const auto detections_from_clusters =
      convert_unassigned_clusters_to_detected_objects(*objs, result);
    m_leftover_publisher->publish(detections_from_clusters);
    maybe_visualize(result.related_rois_stamp, detections_from_clusters);
  } else {
    RCLCPP_WARN(
      get_logger(), "Tracker update for 3D detection at time %d.%d failed. Reason: %s",
      objs->header.stamp.sec, objs->header.stamp.nanosec,
      status_to_string(result.status).c_str());
  }
}


void MultiObjectTrackerNode::odometry_callback(const OdometryMsg::ConstSharedPtr odom)
{
  m_odom_cache->add(odom);
}

void MultiObjectTrackerNode::pose_callback(const PoseMsg::ConstSharedPtr pose)
{
  m_odom_cache->add(to_odom(pose));
}

void MultiObjectTrackerNode::detected_objects_callback(const DetectedObjects::ConstSharedPtr objs)
{
  const rclcpp::Time msg_stamp{objs->header.stamp.sec, objs->header.stamp.nanosec};
  const auto earliest_time = msg_stamp - kMaxLidarEgoStateStampDiff;
  const auto latest_time = msg_stamp + kMaxLidarEgoStateStampDiff;
  const auto matched_msgs = m_odom_cache->getInterval(earliest_time, latest_time);
  if (matched_msgs.empty()) {
    RCLCPP_WARN(get_logger(), "No matching odom msg received for obj msg");
    return;
  }

  DetectedObjectsUpdateResult result;
  mpark::visit(
    [&objs, &matched_msgs, &result](auto & tracker) {
      result = tracker.update(*objs, *get_closest_match(matched_msgs, objs->header.stamp));
    }, m_tracker);

  if (result.status == TrackerUpdateStatus::Ok) {
    m_track_publisher->publish(result.tracks);
    // m_leftover_publisher->publish(result.unassigned_clusters_indices);
    maybe_visualize(result.related_rois_stamp, *objs);
  } else {
    RCLCPP_WARN(
      get_logger(), "Tracker update for 3D detection at time %d.%d failed. Reason: %s",
      objs->header.stamp.sec, objs->header.stamp.nanosec,
      status_to_string(result.status).c_str());
  }
}

void MultiObjectTrackerNode::classified_roi_callback(const ClassifiedRoiArray::ConstSharedPtr rois)
{
  mpark::visit([&rois](auto & tracker) {tracker.update(*rois);}, m_tracker);
}

void MultiObjectTrackerNode::maybe_visualize(
  const builtin_interfaces::msg::Time & rois_stamp,
  DetectedObjects all_objects)
{
  if (!m_visualize_track_creation) {
    return;
  }
  // Align the detections on time with the rois they are associated to.
  all_objects.header.stamp = rois_stamp;
  m_track_creating_clusters_pub->publish(all_objects);
}
}  // namespace tracking_nodes
}  // namespace autoware

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tracking_nodes::MultiObjectTrackerNode)
