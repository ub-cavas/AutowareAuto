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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the multi_object_tracking class.

#ifndef TRACKING__MULTI_OBJECT_TRACKER_HPP_
#define TRACKING__MULTI_OBJECT_TRACKER_HPP_

#include <chrono>
#include <memory>
#include <vector>

#include "autoware_auto_msgs/msg/detected_dynamic_object_array.hpp"
#include "autoware_auto_msgs/msg/tracked_dynamic_object.hpp"
#include "autoware_auto_msgs/msg/tracked_dynamic_object_array.hpp"
#include "kalman_filter/common_states.hpp"
#include "kalman_filter/kalman_filter.hpp"
#include "motion_model/linear_motion_model.hpp"
#include "motion_model/wiener_noise.hpp"
#include "tracking/visibility_control.hpp"


namespace autoware
{
/// \brief Tracking functionality.
namespace tracking
{


/// \brief A return code for the tracker update.
enum class TrackerUpdateStatus
{
  /// Success.
  Ok,
  /// The provided detections were older than the previous detections.
  /// The Kalman filter can only extrapolate forward, so this is an error.
  WentBackInTime,
};


/// \brief Output of MultiObjectTracker::update.
struct TRACKING_PUBLIC TrackerUpdateResult
{
  /// The tracking output. It can be nullptr when the status is not Ok.
  std::unique_ptr<autoware_auto_msgs::msg::TrackedDynamicObjectArray> objects;
  /// Indicates the success or failure, and kind of failure, of the tracking operation.
  TrackerUpdateStatus status;
  /// How many of the input objects are not present in the output.
  int ignored = 0;
};


/// \brief Internal struct containing the object state and other information.
struct TRACKING_LOCAL TrackedObject
{
  /// The state estimator.
  using CA = autoware::prediction::state::ConstAccelerationXY;
  using MotionModel = autoware::prediction::LinearMotionModel<CA>;
  using NoiseModel = autoware::prediction::WienerNoise<CA>;
  using EKF = autoware::prediction::KalmanFilter<MotionModel, NoiseModel>;
  EKF m_ekf;
};


/// \brief Options for object tracking.
struct TRACKING_PUBLIC MultiObjectTrackerOptions
{
  /// Tracks older than this will not be associated with observations.
  std::chrono::milliseconds staleness_threshold = std::chrono::milliseconds(1000);
};


/// \brief A class for multi-object tracking.
class TRACKING_PUBLIC MultiObjectTracker
{
public:
  using DetectedObjects = autoware_auto_msgs::msg::DetectedDynamicObjectArray;
  using TrackedObjects = autoware_auto_msgs::msg::TrackedDynamicObjectArray;

  /// Constructor
  explicit MultiObjectTracker(MultiObjectTrackerOptions options);

  /// \brief Update the tracks with the specified detections and return the tracks at the current
  /// timestamp.
  /// \param[in] detections An array of detections.
  /// \return A result object containing tracks, unless an error occurred.
  TrackerUpdateResult update(const DetectedObjects detections);

private:
  /// Predict forward all the objects to the specified timestamp.
  void predict(std::chrono::system_clock::time_point target_time);

  /// The tracked objects, also called "tracks".
  std::vector<TrackedObject> m_objects;

  /// Timestamp of the last update.
  std::chrono::system_clock::time_point m_last_update;

  /// Configuration values.
  MultiObjectTrackerOptions m_options;
};

}  // namespace tracking
}  // namespace autoware

#endif  // TRACKING__MULTI_OBJECT_TRACKER_HPP_
