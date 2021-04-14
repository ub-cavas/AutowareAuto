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

#include "gtest/gtest.h"
#include "autoware_auto_msgs/msg/detected_dynamic_object_array.hpp"
#include "tracking/multi_object_tracker.hpp"

using Tracker = autoware::tracking::MultiObjectTracker;
using Options = autoware::tracking::MultiObjectTrackerOptions;
using Status = autoware::tracking::TrackerUpdateStatus;
using DetectedObjects = autoware_auto_msgs::msg::DetectedDynamicObjectArray;

TEST(test_multi_object_tracker, test_timestamps) {
  Tracker tracker{Options {}};
  DetectedObjects detections;
  detections.header.stamp.sec = 1000;
  tracker.update(detections);
  detections.header.stamp.sec = 999;
  auto result = tracker.update(detections);
  EXPECT_EQ(result.status, Status::WentBackInTime);
}
