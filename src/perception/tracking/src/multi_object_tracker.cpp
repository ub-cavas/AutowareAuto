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
#include <memory>

#include "time_utils/time_utils.hpp"

namespace autoware
{
namespace perception
{
namespace tracking
{

MultiObjectTracker::MultiObjectTracker(MultiObjectTrackerOptions options)
: m_options(options), m_associator(options.assoc_config) {}

TrackerUpdateResult MultiObjectTracker::update(const DetectedObjectsMsg & detections)
{
  TrackerUpdateStatus validation_status = this->validate(detections);
  TrackerUpdateResult result;
  if (validation_status != TrackerUpdateStatus::Ok) {
    result.status = TrackerUpdateStatus::WentBackInTime;
    return result;
  }
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
  return result;
}


TrackerUpdateStatus MultiObjectTracker::validate(const DetectedObjectsMsg & detections)
{
  const auto target_time = time_utils::from_message(detections.header.stamp);
  if (target_time < m_last_update) {
    return TrackerUpdateStatus::WentBackInTime;
  }
  if (detections.header.frame_id != m_options.frame) {
    return TrackerUpdateStatus::WrongFrame;
  }
  // Could also validate classes, and object shapes
  m_last_update = target_time;
  return TrackerUpdateStatus::Ok;
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
