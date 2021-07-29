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

#include "gtest/gtest.h"
#include "autoware_auto_msgs/msg/detected_objects.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tracking/multi_object_tracker.hpp"

using Tracker = autoware::perception::tracking::MultiObjectTracker;
using Options = autoware::perception::tracking::MultiObjectTrackerOptions;
using Status = autoware::perception::tracking::TrackerUpdateStatus;
using DetectedObjects = autoware_auto_msgs::msg::DetectedObjects;
using DetectedObjectMsg = autoware_auto_msgs::msg::DetectedObject;
using Odometry = nav_msgs::msg::Odometry;

class MultiObjectTrackerTest : public ::testing::Test
{

private:
  void init_shape()
  {
    DetectedObjectMsg detection;
    detection.kinematics.centroid_position.x = 0.0;
    detection.kinematics.centroid_position.y = 0.0;
    geometry_msgs::msg::Point32 pt1;
    pt1.x = -1.0F;
    pt1.y = -1.0F;
    pt1.z = 0.0F;
    geometry_msgs::msg::Point32 pt2;
    pt2.x = -1.0F;
    pt2.y = 1.0F;
    pt2.z = 0.0F;
    geometry_msgs::msg::Point32 pt3;
    pt3.x = 1.0F;
    pt3.y = -1.0F;
    pt3.z = 0.0F;
    geometry_msgs::msg::Point32 pt4;
    pt4.x = 1.0F;
    pt4.y = 1.0F;
    pt4.z = 0.0F;
    detection.shape.polygon.points.push_back(pt4);
    detection.shape.polygon.points.push_back(pt3);
    detection.shape.polygon.points.push_back(pt1);
    detection.shape.polygon.points.push_back(pt2);
    m_detections.objects.push_back(detection);
  }

public:
  MultiObjectTrackerTest()
  : m_tracker{Options{{2.0F, 2.5F}, 1.0F, 1.0F}}
  {
    m_detections.header.frame_id = "base_link";
    m_detections.header.stamp.sec = 1000;
    init_shape();
    m_odom.header.frame_id = "map";
    m_odom.child_frame_id = "base_link";
    m_odom.header.stamp.sec = 1000;
    m_odom.pose.pose.orientation.w = 1.0;
  }

  void SetUp() {}

  void TearDown() {}

  Tracker m_tracker;
  DetectedObjects m_detections;
  Odometry m_odom;
};

TEST_F(MultiObjectTrackerTest, test_happy_path) {
  m_tracker = Tracker({Options{{2.0F, 2.5F}, 1.0F, 1.0F}});
  EXPECT_NO_THROW(m_tracker.update(m_detections, m_odom));
}

TEST_F(MultiObjectTrackerTest, test_timestamps) {
  m_tracker = Tracker({Options{{2.0F, 2.5F}, 1.0F, 1.0F}});
  m_tracker.update(m_detections, m_odom);
  m_detections.header.stamp.sec = 999;
  const auto result = m_tracker.update(m_detections, m_odom);
  EXPECT_EQ(result.status, Status::WentBackInTime);
}

TEST_F(MultiObjectTrackerTest, test_frame_orientation_validation) {
  m_tracker = Tracker({Options{{2.0F, 2.5F}, 1.0F, 1.0F}});
  // This rotates the object to be on its side, which does not make sense
  m_odom.pose.pose.orientation.w = 0.0;
  m_odom.pose.pose.orientation.x = 1.0;
  const auto result = m_tracker.update(m_detections, m_odom);
  EXPECT_EQ(result.status, Status::FrameNotGravityAligned);
}


TEST_F(MultiObjectTrackerTest, test_distance_threshold_detection_track) {

  m_tracker = Tracker({Options{{2.0F, 2.5F}, 0.01F, 1.0F}});
  const auto result = m_tracker.update(m_detections, m_odom);
  ASSERT_EQ(result.objects->objects.size(), 1U);

  // If the new detection is inside the distance threshold
  m_detections.objects[0].kinematics.centroid_position.x = 1.0;
  m_detections.objects[0].kinematics.centroid_position.y = 0.0;
  const auto result_2 = m_tracker.update(m_detections, m_odom);
  ASSERT_EQ(result_2.objects->objects.size(), 1U);

  // If the new detection is outside the distance threshold it will be considered
  // as unassociated and a new track will be created
  m_detections.objects[0].kinematics.centroid_position.x = 10.0;
  m_detections.objects[0].kinematics.centroid_position.y = 0.0;
  const auto result_3 = m_tracker.update(m_detections, m_odom);
  ASSERT_EQ(result_3.objects->objects.size(), 2U);

}


TEST_F(MultiObjectTrackerTest, test_area_threshold_detection_track) {

  m_tracker = Tracker({Options{{2.0F, 10.0F}, 1.0F, 1.0F}});
  const auto result = m_tracker.update(m_detections, m_odom);
  ASSERT_EQ(result.objects->objects.size(), 1U);

  // If the new detection is inside the area threshold
  for (auto & pt : m_detections.objects[0].shape.polygon.points) {
    pt.x = pt.x * 0.5F;
    pt.y = pt.y * 0.5F;
  }
  const auto result_3 = m_tracker.update(m_detections, m_odom);
  ASSERT_EQ(result_3.objects->objects.size(), 1U);

  // If the new detection is outside the area threshold it will be considered
  // as unassociated and a new track will be created
  for (auto & pt : m_detections.objects[0].shape.polygon.points) {
    pt.x = pt.x * 10.F;
    pt.y = pt.y * 10.F;
  }
  const auto result_2 = m_tracker.update(m_detections, m_odom);
  ASSERT_EQ(result_2.objects->objects.size(), 2U);
}
