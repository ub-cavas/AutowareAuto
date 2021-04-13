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

#include "multi_object_tracking/multi_object_tracking.hpp"

#include "time_utils/time_utils.hpp"

namespace autoware
{

namespace tracking
{

MultiObjectTracker::MultiObjectTracker(MultiObjectTrackerOptions options)
: m_options(options) {}

TrackerUpdateResult MultiObjectTracker::update(const DetectedObjects detections)
{
  TrackerUpdateResult result;
  auto target_time = time_utils::from_message(detections.header.stamp);
  if (target_time < m_last_update) {
    result.status = TrackerUpdateStatus::WentBackInTime;
    return result;
  }
  this->predict(target_time);

  /// Associate, etc.
  throw std::runtime_error("Not yet implemented");

  m_last_update = target_time;
  return result;
}

void MultiObjectTracker::predict(std::chrono::system_clock::time_point target_time)
{
  const auto dt = target_time - m_last_update;
  for (auto & object : m_objects) {
    object.m_ekf.predict(dt);
  }
}


}  // namespace tracking

}  // namespace autoware
