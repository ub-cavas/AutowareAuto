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

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tracking/multi_object_tracker.hpp>

#include <memory>

using autoware::perception::tracking::LidarOnlyPolicy;
using autoware::perception::tracking::MultiObjectTracker;
using autoware::perception::tracking::TrackCreationPolicy;
using autoware::perception::tracking::TrackCreator;
using autoware::perception::tracking::MultiObjectTrackerOptions;
using autoware::perception::tracking::TrackerUpdateStatus;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using autoware_auto_perception_msgs::msg::PointClusters;
using nav_msgs::msg::Odometry;

class MultiObjectTrackerTest : public ::testing::Test
{
public:
  using Policy = LidarOnlyPolicy;
  using TrackCreatorType = TrackCreator<Policy>;

  MultiObjectTrackerTest()
  : m_policy{std::make_shared<Policy>(1.0, 1.0, m_tf_buffer)},
    m_track_creator{m_policy},
    m_tracker{MultiObjectTrackerOptions {{2.0F, 2.5F, true}, {}}, m_track_creator, m_tf_buffer}
  {
    m_detections.header.frame_id = "base_link";
    m_detections.header.stamp.sec = 1000;
    m_odom.header.frame_id = "map";
    m_odom.child_frame_id = "base_link";
    m_odom.header.stamp.sec = 1000;
    m_odom.pose.pose.orientation.w = 1.0;
  }

  void SetUp() {}

  void TearDown() {}

  tf2::BufferCore m_tf_buffer;
  std::shared_ptr<Policy> m_policy;
  TrackCreatorType m_track_creator;
  MultiObjectTracker<TrackCreatorType> m_tracker;
  DetectedObjects m_detections;
  PointClusters m_clusters;
  Odometry m_odom;
};

TEST_F(MultiObjectTrackerTest, TestHappyPathDetections) {
  EXPECT_NO_THROW(m_tracker.update(m_detections, m_odom));
}

TEST_F(MultiObjectTrackerTest, TestHappyPathClusters) {
  EXPECT_NO_THROW(m_tracker.update(m_clusters, m_odom));
}

TEST_F(MultiObjectTrackerTest, TestHappyPathDetectionsAndClusters) {
  EXPECT_NO_THROW(m_tracker.update(m_detections, m_clusters, m_odom));
}

TEST_F(MultiObjectTrackerTest, TestTimestamps) {
  m_tracker.update(m_detections, m_odom);
  m_detections.header.stamp.sec = 999;
  const auto result = m_tracker.update(m_detections, m_odom);
  EXPECT_EQ(result.status, TrackerUpdateStatus::WentBackInTime);
}

TEST_F(MultiObjectTrackerTest, TestFrameOrientationValidation) {
  // This rotates the object to be on its side, which does not make sense
  m_odom.pose.pose.orientation.w = 0.0;
  m_odom.pose.pose.orientation.x = 1.0;
  const auto result = m_tracker.update(m_detections, m_odom);
  EXPECT_EQ(result.status, TrackerUpdateStatus::FrameNotGravityAligned);
}

TEST_F(MultiObjectTrackerTest, TestDetectionFrameValidation) {
  m_detections.header.frame_id = "test_frame";
  const auto result = m_tracker.update(m_detections, m_clusters, m_odom);
  EXPECT_EQ(result.status, TrackerUpdateStatus::DetectionFrameMismatch);
}

TEST_F(MultiObjectTrackerTest, TestTrackerFrameMismatch) {
  m_odom.header.frame_id = "test_frame";
  const auto result = m_tracker.update(m_detections, m_clusters, m_odom);
  EXPECT_EQ(result.status, TrackerUpdateStatus::TrackerFrameMismatch);
}
