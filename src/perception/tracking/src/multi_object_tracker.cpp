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

#include "tracking/multi_object_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "time_utils/time_utils.hpp"
#include "tf2_eigen/tf2_eigen.h"

namespace autoware
{
namespace perception
{
namespace tracking
{

MultiObjectTracker::MultiObjectTracker(MultiObjectTrackerOptions options)
: m_options(options), m_associator(options.assoc_config) {}

TrackerUpdateResult MultiObjectTracker::update(
  DetectedObjectsMsg detections,
  const geometry_msgs::msg::TransformStamped & track_from_detection)
{
  TrackerUpdateStatus validation_status = this->validate(detections, track_from_detection);
  TrackerUpdateResult result;
  if (validation_status != TrackerUpdateStatus::Ok) {
    result.status = TrackerUpdateStatus::WentBackInTime;
    return result;
  }

  // ==================================
  // Transform detections
  // ==================================
  this->transform(detections, track_from_detection);

  // ==================================
  // Predict tracks forward
  // ==================================
  // TODO(nikolai.morin): Simplify after #1002
  const auto target_time = time_utils::from_message(detections.header.stamp);
  const auto dt = target_time - m_last_update;
  for (auto & object : m_objects) {
    object.predict(dt);
  }

  // ==================================
  // Associate observations with tracks
  // ==================================
  TrackedObjectsMsg tracked_objects_msg = this->convert_to_msg();
  AssociatorResult association;
  try {
    association = m_associator.assign(detections, tracked_objects_msg);
  } catch (const std::runtime_error & e) {
    result.status = TrackerUpdateStatus::InvalidShape;
    return result;
  }

  // ==================================
  // Update tracks with observations
  // ==================================
  for (size_t track_idx = 0; track_idx < m_objects.size(); ++track_idx) {
    size_t detection_idx = association.track_assignments[track_idx];
    if (detection_idx == AssociatorResult::UNASSIGNED) {
      continue;
    }
    const auto & detection = detections.objects[detection_idx];
    m_objects[track_idx].update(detection);
  }

  // ==================================
  // Initialize new tracks
  // ==================================
  for (size_t new_detection_idx : association.unassigned_detection_indices) {
    // TODO(nikolai.morin): Make parameters configurable
    m_objects.push_back(
      TrackedObject(
        detections.objects[new_detection_idx],
        m_options.default_variance, m_options.noise_variance));
  }

  // ==================================
  // Prune tracks
  // ==================================
  // TODO(nikolai.morin): Implement pruning rule

  // ==================================
  // Build result
  // ==================================
  result.objects = std::make_unique<TrackedObjectsMsg>(this->convert_to_msg());
  result.status = TrackerUpdateStatus::Ok;
  m_last_update = target_time;
  return result;
}


TrackerUpdateStatus MultiObjectTracker::validate(
  const DetectedObjectsMsg & detections,
  const geometry_msgs::msg::TransformStamped & track_from_detection)
{
  const auto target_time = time_utils::from_message(detections.header.stamp);
  if (target_time < m_last_update) {
    return TrackerUpdateStatus::WentBackInTime;
  }
  if (detections.header.stamp != track_from_detection.header.stamp) {
    return TrackerUpdateStatus::TimestampMismatch;
  }
  if (detections.header.frame_id != track_from_detection.header.frame_id) {
    return TrackerUpdateStatus::DetectionFrameMismatch;
  }
  if (track_from_detection.child_frame_id != m_options.frame) {
    return TrackerUpdateStatus::TrackerFrameMismatch;
  }
  // Check that the transformation is still roughly 2D, i.e. does not have substantial pitch and
  // roll. That means that either the rotation angle is small, or the rotation axis is
  // approximately equal to the z axis.
  constexpr double kAngleThresh = 0.1;  // rad
  constexpr double kAxisTiltThresh = 0.1;  // rad
  // rotation angle small
  // ⇔ |θ| <= kAngleThresh  (angles are assumed to be between -π and π)
  // ⇔ cos(θ/2) => std::cos(kAngleThresh/2)
  // ⇔ w => std::cos(kAngleThresh/2)
  const auto & quat = track_from_detection.transform.rotation;
  if (quat.w < std::cos(0.5 * kAngleThresh)) {
    // From Wikipedia: (x, y, z) = cos(θ/2) * (u_x, u_y, u_z), where u is the rotation axis.
    // The cosine of the angle α between the rotation axis and the z axis is the dot product of the
    // rotation axis u and the the z axis, so cos(α) = u_z.
    const double u_z = quat.z / std::sqrt(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z);
    if (u_z < std::cos(kAxisTiltThresh)) {
      return TrackerUpdateStatus::FrameNotGravityAligned;
    }
  }

  // Could also validate classes, and object shapes
  return TrackerUpdateStatus::Ok;
}

void MultiObjectTracker::transform(
  DetectedObjectsMsg & detections,
  const geometry_msgs::msg::TransformStamped & track_from_detection)
{
  const Eigen::Isometry3d transform_d = tf2::transformToEigen(track_from_detection);
  const Eigen::Isometry3f transform_f = transform_d.cast<float>();
  const Eigen::Matrix3d rot_d = transform_d.linear();
  // Hoisted outside the loop
  Eigen::Isometry3d obj_from_detection_frame = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d obj_from_tracking_frame = Eigen::Isometry3d::Identity();
  detections.header.frame_id = m_options.frame;
  for (auto & detection : detections.objects) {
    for (auto & point : detection.shape.polygon.points) {
      Eigen::Vector3f eigen_point {point.x, point.y, point.z};
      Eigen::Vector3f eigen_point_transformed = transform_f * eigen_point;
      point.x = eigen_point_transformed.x();
      point.y = eigen_point_transformed.y();
      point.z = eigen_point_transformed.z();
    }
    if (detection.kinematics.has_pose) {
      tf2::fromMsg(detection.kinematics.pose.pose, obj_from_detection_frame);
      obj_from_tracking_frame = transform_d * obj_from_detection_frame;
      detection.kinematics.pose.pose = tf2::toMsg(obj_from_tracking_frame);
      if (detection.kinematics.has_pose_covariance) {
        // Doing this properly is difficult. We'll ignore the rotational part. This is a practical
        // solution since only the yaw covariance is relevant, and the yaw covariance is
        // unaffected by the transformation, which preserves the z axis.
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> cov(
          detection.kinematics.pose.covariance.data());
        cov.topLeftCorner<3, 3>() = rot_d * cov.topLeftCorner<3, 3>() * rot_d.transpose();
      }
    }
    // TODO(nikolai.morin): Need to take the twist of the detection frame as input and transform
    // the twist.
    throw std::runtime_error("Twist transform is not implemented yet");
  }
}

MultiObjectTracker::TrackedObjectsMsg MultiObjectTracker::convert_to_msg() const
{
  TrackedObjectsMsg array;
  array.objects.reserve(m_objects.size());
  std::transform(
    m_objects.begin(), m_objects.end(), std::back_inserter(array.objects), [](
      TrackedObject o) {return o.msg();});
  return array;
}


}  // namespace tracking
}  // namespace perception
}  // namespace autoware
