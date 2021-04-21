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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tracking/multi_object_tracker.hpp"

using Tracker = autoware::perception::tracking::MultiObjectTracker;
using Options = autoware::perception::tracking::MultiObjectTrackerOptions;
using Status = autoware::perception::tracking::TrackerUpdateStatus;
using DetectedObjects = autoware_auto_msgs::msg::DetectedDynamicObjectArray;
using TransformStamped = geometry_msgs::msg::TransformStamped;

class MultiObjectTrackerTest : public ::testing::Test
{
public:
  MultiObjectTrackerTest()
  : m_tracker{Options{}}
  {
    m_detections.header.stamp.sec = 1000;
    m_tf.header.stamp.sec = 1000;
  }

  void SetUp() {}

  void TearDown() {}

  Tracker m_tracker;
  DetectedObjects m_detections;
  TransformStamped m_tf;
};

TEST_F(MultiObjectTrackerTest, test_timestamps) {
  m_tracker.update(m_detections, m_tf);
  m_detections.header.stamp.sec = 999;
  auto result = m_tracker.update(m_detections, m_tf);
  EXPECT_EQ(result.status, Status::WentBackInTime);
}
