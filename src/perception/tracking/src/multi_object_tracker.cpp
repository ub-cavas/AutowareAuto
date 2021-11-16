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

#include <tracking/multi_object_tracker.hpp>

#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>
#include <geometry/bounding_box/bounding_box_common.hpp>
#include <geometry/bounding_box_2d.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <lidar_utils/cluster_utils/point_clusters_view.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <time_utils/time_utils.hpp>

#include <algorithm>
#include <cmath>
#include <list>
#include <memory>


namespace autoware
{
namespace perception
{
namespace tracking
{
namespace
{
using Classification = autoware_auto_perception_msgs::msg::ObjectClassification;
using ClassificationsType =
  autoware_auto_perception_msgs::msg::DetectedObject::_classification_type;
static const ClassificationsType kUnknownClass = {
  Classification{}.set__classification(Classification::UNKNOWN).set__probability(1.0F)};
static constexpr float32_t KLidarClassificationCovariance = 1.0F;
static constexpr float32_t kVisionClassificationCovariance = 0.1F;

using autoware::common::types::float64_t;
using geometry_msgs::build;
using geometry_msgs::msg::Point32;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using autoware_auto_perception_msgs::msg::DetectedObjectKinematics;
using nav_msgs::msg::Odometry;
using TrackedObjectsMsg = autoware_auto_perception_msgs::msg::TrackedObjects;

bool is_gravity_aligned(const geometry_msgs::msg::Quaternion & quat)
{
  // Check that the transformation is still roughly 2D, i.e. does not have substantial pitch and
  // roll. That means that either the rotation angle is small, or the rotation axis is
  // approximately equal to the z axis.
  constexpr float64_t kAngleThresh = 0.1;  // rad
  constexpr float64_t kAxisTiltThresh = 0.1;  // rad
  // rotation angle small
  // ⇔ |θ| <= kAngleThresh  (angles are assumed to be between -π and π)
  // ⇔ cos(θ/2) => std::cos(kAngleThresh/2)
  // ⇔ w => std::cos(kAngleThresh/2)
  if (quat.w < std::cos(0.5 * kAngleThresh)) {
    // From Wikipedia: (x, y, z) = cos(θ/2) * (u_x, u_y, u_z), where u is the rotation axis.
    // The cosine of the angle α between the rotation axis and the z axis is the dot product of the
    // rotation axis u and the the z axis, so cos(α) = u_z.
    const float64_t u_z = std::abs(quat.z) / std::sqrt(
      quat.x * quat.x + quat.y * quat.y + quat.z * quat.z);
    if (u_z < std::cos(kAxisTiltThresh)) {
      return false;
    }
  }
  return true;
}

geometry_msgs::msg::TransformStamped to_transform(const Odometry & odometry)
{
  geometry_msgs::msg::TransformStamped tfs;
  tfs.header = odometry.header;
  tfs.child_frame_id = odometry.child_frame_id;
  tfs.transform.translation.x = odometry.pose.pose.position.x;
  tfs.transform.translation.y = odometry.pose.pose.position.y;
  tfs.transform.translation.z = odometry.pose.pose.position.z;
  tfs.transform.rotation = odometry.pose.pose.orientation;
  return tfs;
}

DetectedObjects transform(
  const std_msgs::msg::Header::_frame_id_type & target_frame_id,
  const DetectedObjects & detections,
  const Odometry & detection_frame_odometry)
{
  // Convert the odometry to Eigen objects.
  Eigen::Isometry3d tf__tracking__detection = Eigen::Isometry3d::Identity();
  tf2::fromMsg(detection_frame_odometry.pose.pose, tf__tracking__detection);
  const Eigen::Matrix3d rot_d = tf__tracking__detection.linear();
  // Convert the odometry to TransformStamped for use with tf2::doTransform.
  const geometry_msgs::msg::TransformStamped tf_msg__tracking__detection = to_transform(
    detection_frame_odometry);
  // Hoisted outside the loop
  Eigen::Vector3d centroid_detection = Eigen::Vector3d::Zero();

  DetectedObjects result;
  result.header = detections.header;
  result.header.frame_id = target_frame_id;
  result.objects.reserve(detections.objects.size());
  for (const auto & original_detection : detections.objects) {
    result.objects.emplace_back(original_detection);
    auto & detection = result.objects.back();
    // Transform the pose.
    tf2::fromMsg(detection.kinematics.centroid_position, centroid_detection);
    const Eigen::Vector3d centroid_tracking = tf__tracking__detection * centroid_detection;
    detection.kinematics.centroid_position = tf2::toMsg(centroid_tracking);
    if (detection.kinematics.orientation_availability != DetectedObjectKinematics::UNAVAILABLE) {
      geometry_msgs::msg::QuaternionStamped q_out;
      // Use quaternion stamped because there is no doTransform for quaternion even though
      // stamp of QuaternionStamped is not being used for anything
      tf2::doTransform(
        geometry_msgs::msg::QuaternionStamped{}.set__quaternion(detection.kinematics.orientation),
        q_out,
        tf_msg__tracking__detection);
      detection.kinematics.orientation = q_out.quaternion;
    }
    if (detection.kinematics.has_position_covariance) {
      // Doing this properly is difficult. We'll ignore the rotational part. This is a practical
      // solution since only the yaw covariance is relevant, and the yaw covariance is
      // unaffected by the transformation, which preserves the z axis.
      // An even more accurate implementation could additionally include the odometry covariance.
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> cov(
        detection.kinematics.position_covariance.data());
      cov = rot_d * cov * rot_d.transpose();
    }
    // Transform the twist.
    if (detection.kinematics.has_twist) {
      auto & linear = detection.kinematics.twist.twist.linear;
      const auto & frame_linear = detection_frame_odometry.twist.twist.linear;
      const Eigen::Vector3d eigen_linear{linear.x, linear.y, linear.z};
      const Eigen::Vector3d eigen_linear_transformed = rot_d * eigen_linear;
      // This assumes the detection frame has no angular velocity wrt the tracking frame.
      // TODO(nikolai.morin): Implement the full formula, to be found in
      // Craig's "Introduction to robotics" book, third edition, formula 5.13
      linear.x = frame_linear.x + eigen_linear_transformed.x();
      linear.y = frame_linear.y + eigen_linear_transformed.y();
      linear.z = frame_linear.z + eigen_linear_transformed.z();
    }
  }
  return result;
}

TrackedObjectsMsg convert_to_msg(
  const TrackedObjects & tracks, const builtin_interfaces::msg::Time & stamp)
{
  TrackedObjectsMsg array;
  array.header.stamp = stamp;
  array.header.frame_id = tracks.frame_id;
  array.objects.reserve(tracks.objects.size());
  std::transform(
    tracks.objects.begin(), tracks.objects.end(), std::back_inserter(array.objects), [](
      TrackedObject o) {return o.msg();});
  return array;
}


}  // anonymous namespace

template<class TrackCreatorT>
MultiObjectTracker<TrackCreatorT>::MultiObjectTracker(
  MultiObjectTrackerOptions options,
  TrackCreatorT track_creator,
  const tf2::BufferCore & buffer)
: m_options{options},
  m_object_associator{options.object_association_config},
  m_vision_associator{options.vision_association_config, buffer},
  m_track_creator{track_creator}
{
  m_tracks.frame_id = m_options.frame;
}

template<class TrackCreatorT>
DetectedObjectsUpdateResult MultiObjectTracker<TrackCreatorT>::update(
  const ClustersMsg & incoming_clusters,
  const nav_msgs::msg::Odometry & detection_frame_odometry)
{
  ClustersMsg clusters = incoming_clusters;
  BoundingBoxArray boxes_in_detection_frame;
  boxes_in_detection_frame.header.stamp = clusters.header.stamp;
  boxes_in_detection_frame.header.frame_id = clusters.header.frame_id;
  for (auto cls_id = 0U; cls_id < clusters.cluster_boundary.size(); cls_id++) {
    const auto iter_pair = common::lidar_utils::get_cluster(clusters, cls_id);
    if (iter_pair.first == iter_pair.second) {
      continue;
    }
    boxes_in_detection_frame.boxes.push_back(
      common::geometry::bounding_box::lfit_bounding_box_2d(iter_pair.first, iter_pair.second));
    common::geometry::bounding_box::compute_height(
      iter_pair.first, iter_pair.second, boxes_in_detection_frame.boxes.back());
  }

  DetectedObjects detections;
  detections.objects.reserve(boxes_in_detection_frame.boxes.size());
  detections.header = boxes_in_detection_frame.header;
  std::transform(
    boxes_in_detection_frame.boxes.begin(),
    boxes_in_detection_frame.boxes.end(),
    std::back_inserter(detections.objects),
    common::geometry::bounding_box::details::make_detected_object);

  return update(detections, detection_frame_odometry);
}

/// \relates autoware::perception::tracking::MultiObjectTracker
template<class TrackCreatorT>
DetectedObjectsUpdateResult MultiObjectTracker<TrackCreatorT>::update(
  const DetectedObjects & detections,
  const nav_msgs::msg::Odometry & detection_frame_odometry)
{
  DetectedObjectsUpdateResult result;
  result.status = this->validate(detections, detection_frame_odometry);
  if (result.status != TrackerUpdateStatus::Ok) {
    return result;
  }

  // ==================================
  // Transform detections
  // ==================================
  DetectedObjects detections_in_tracker_frame{
    transform(m_options.frame, detections, detection_frame_odometry)};

  // ==================================
  // Predict tracks forward
  // ==================================
  // TODO(nikolai.morin): Simplify after #1002
  const auto target_time = time_utils::from_message(detections.header.stamp);
  const auto dt = target_time - m_last_update;
  for (auto & object : m_tracks.objects) {
    object.predict(dt);
  }

  // ==================================
  // Associate observations with tracks
  // ==================================
  const auto detection_associations =
    m_object_associator.assign(detections_in_tracker_frame, this->m_tracks);
  ObjectsWithAssociations detections_with_associations{
    detections_in_tracker_frame, detection_associations};

  // ==================================
  // Update tracks with observations
  // ==================================
  for (size_t detection_idx = 0;
    detection_idx < detections_with_associations.associations().size();
    ++detection_idx)
  {
    const auto & association = detections_with_associations.associations()[detection_idx];
    if (association.matched == Matched::kExistingTrack) {
      const auto & detection = detections_with_associations.objects().objects[detection_idx];
      const auto matched_track_index = association.match_index;
      m_tracks.objects[matched_track_index].update(detection);
    }
  }
  const auto & track_associations = m_object_associator.track_associations();
  for (auto idx = 0U; idx < m_object_associator.track_associations().size(); ++idx) {
    const auto & association = track_associations[idx];
    if (association.matched == Matched::kNothing) {
      m_tracks.objects[idx].no_update();
    }
  }

  // ==================================
  // Initialize new tracks
  // ==================================
  const auto track_creation_result = m_track_creator.create_tracks(detections_with_associations);
  m_tracks.objects.insert(
    m_tracks.objects.end(),
    std::make_move_iterator(track_creation_result.tracks.begin()),
    std::make_move_iterator(track_creation_result.tracks.end()));
  result.related_rois_stamp = track_creation_result.related_rois_stamp;
  // Set all the unassigned cluster indices to the output.
  for (auto i = 0U; i < track_creation_result.associations.size(); ++i) {
    if (track_creation_result.associations[i].matched == Matched::kNothing) {
      result.unassigned_clusters_indices.push_back(i);
    }
  }

  // ==================================
  // Prune tracks
  // ==================================
  // TODO(igor): If we ever need to ensure that the indices of tracks are correct in the association
  // array that is part of the track creation result, we would need to make sure we adapt the
  // indices in that array once some tracks are removed.
  const auto last = std::remove_if(
    m_tracks.objects.begin(), m_tracks.objects.end(), [this](const auto & object) {
      return object.should_be_removed(
        this->m_options.pruning_time_threshold,
        this->m_options.pruning_ticks_threshold);
    });
  m_tracks.objects.erase(last, m_tracks.objects.end());
  // ==================================
  // Build result
  // ==================================
  result.tracks = convert_to_msg(m_tracks, detections_with_associations.objects().header.stamp);
  result.status = TrackerUpdateStatus::Ok;
  m_last_update = target_time;

  return result;
}

template<class TrackCreatorT>
void MultiObjectTracker<TrackCreatorT>::update(const ClassifiedRoiArrayMsg & rois)
{
  const auto target_time = time_utils::from_message(rois.header.stamp);
  if (target_time < m_last_update) {
    std::cerr << "Trying to update the tracks in the past. Skipping." << std::endl;
    return;
  }
  const auto dt = target_time - m_last_update;
  auto tracks_copy = m_tracks;
  for (auto & object : tracks_copy.objects) {
    object.predict(dt);
  }
  const auto association = m_vision_associator.assign(rois, tracks_copy);
  // Update the original tracks' classification.
  for (size_t i = 0U; i < m_tracks.objects.size(); ++i) {
    const auto & maybe_roi_idx = association.track_assignments[i];
    if (maybe_roi_idx != AssociatorResult::UNASSIGNED) {
      m_tracks.objects[i].update(
        rois.rois[maybe_roi_idx].classifications,
        kVisionClassificationCovariance);
    } else {
      m_tracks.objects[i].update(kUnknownClass, KLidarClassificationCovariance);
    }
  }
  m_track_creator.add_objects(rois, association);
}

template<class TrackCreatorT>
TrackerUpdateStatus MultiObjectTracker<TrackCreatorT>::validate(
  const DetectedObjects & detections,
  const nav_msgs::msg::Odometry & detection_frame_odometry)
{
  const auto target_time = time_utils::from_message(detections.header.stamp);
  if (target_time < m_last_update) {
    return TrackerUpdateStatus::WentBackInTime;
  }
  if (detections.header.frame_id != detection_frame_odometry.child_frame_id) {
    return TrackerUpdateStatus::DetectionFrameMismatch;
  }
  if (detection_frame_odometry.header.frame_id != m_options.frame) {
    return TrackerUpdateStatus::TrackerFrameMismatch;
  }
  if (!is_gravity_aligned(detection_frame_odometry.pose.pose.orientation)) {
    return TrackerUpdateStatus::FrameNotGravityAligned;
  }
  // Could also validate
  // * classes
  // * object shapes
  // * detection poses are gravity aligned
  return TrackerUpdateStatus::Ok;
}

// Disabling this warning is required to enable an explicit function instantiation in the header. I
// believe this is a bug in the compiler ¯\_(ツ)_/¯
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
template class TRACKING_PUBLIC MultiObjectTracker<TrackCreator<LidarOnlyPolicy>>;
template class TRACKING_PUBLIC MultiObjectTracker<TrackCreator<LidarClusterIfVisionPolicy>>;
#pragma GCC diagnostic pop

}  // namespace tracking
}  // namespace perception
}  // namespace autoware
