// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <gtest/gtest.h>

#include <tracking/detected_object_associator.hpp>

#include <vector>

using TrackedObjects = autoware_auto_perception_msgs::msg::TrackedObjects;
using TrackedObject = autoware_auto_perception_msgs::msg::TrackedObject;

using DetectedObjects = autoware_auto_perception_msgs::msg::DetectedObjects;
using DetectedObject = autoware_auto_perception_msgs::msg::DetectedObject;

namespace tracking = autoware::perception::tracking;

namespace
{
constexpr auto kTrackerFrame = "odom";
}  // namespace

class AssociationTester : public testing::Test
{
protected:
  AssociationTester()
  : m_association_cfg(10.0F, 2.0F, true),
    m_associator(m_association_cfg)
  {
    // just set x and y fields of the covariance since only that is used in the associator
    m_some_covariance[0] = 0.5;
    m_some_covariance[1] = -0.09;
    m_some_covariance[6] = 1.0;
    m_some_covariance[7] = 0.09;
  }

  // Square will be centered on origin since vertices are not used for any check except area
  autoware_auto_perception_msgs::msg::Shape create_square(float area)
  {
    autoware_auto_perception_msgs::msg::Shape shape;
    const float side_length = std::sqrt(area);

    shape.polygon.points.push_back(
      geometry_msgs::msg::Point32{}.set__x(0.0F).set__y(0.0F)
      .set__z(0.0F));
    shape.polygon.points.push_back(
      geometry_msgs::msg::Point32{}.set__x(side_length).set__y(0.0F)
      .set__z(0.0F));
    shape.polygon.points.push_back(
      geometry_msgs::msg::Point32{}.set__x(side_length).set__y(side_length)
      .set__z(0.0F));
    shape.polygon.points.push_back(
      geometry_msgs::msg::Point32{}.set__x(0.0F).set__y(side_length)
      .set__z(0.0F));
    return shape;
  }

  std::array<double, 36> m_some_covariance;
  tracking::DataAssociationConfig m_association_cfg;
  tracking::DetectedObjectAssociator m_associator;
};


// Two objects, one track. Track has huge y variance and small x variance.
// Object 1 is shorter distance away on x, same y. Object2 is longer distance away on Y,
// same x. Associator should associate track with object2
TEST_F(AssociationTester, Basic)
{
  TrackedObjects tracks_msg;
  std::vector<tracking::TrackedObject> tracked_object_vec{};
  DetectedObject track1_obj;
  track1_obj.shape = create_square(4.0F);
  track1_obj.kinematics.pose_with_covariance.pose.position.x = 2.0;
  track1_obj.kinematics.pose_with_covariance.pose.position.y = 2.0;
  track1_obj.kinematics.pose_with_covariance.covariance[0] = 0.5;
  track1_obj.kinematics.pose_with_covariance.covariance[1] = -0.09;
  track1_obj.kinematics.pose_with_covariance.covariance[6] = -0.09;
  track1_obj.kinematics.pose_with_covariance.covariance[7] = 10.43;
  track1_obj.kinematics.has_position_covariance = true;

  tracked_object_vec.emplace_back(track1_obj, 0.0, 0.0);

  DetectedObjects objects_msg;
  DetectedObject obj1;
  obj1.shape = create_square(4.0F);
  obj1.kinematics.pose_with_covariance.pose.position.x = 2.5;
  obj1.kinematics.pose_with_covariance.pose.position.y = 2.0;
  obj1.kinematics.pose_with_covariance.covariance[0] = 0.5;
  obj1.kinematics.pose_with_covariance.covariance[1] = -0.09;
  obj1.kinematics.pose_with_covariance.covariance[6] = -0.09;
  obj1.kinematics.pose_with_covariance.covariance[7] = 10.43;
  objects_msg.objects.push_back(obj1);

  DetectedObject obj2;
  obj2 = obj1;
  obj2.kinematics.pose_with_covariance.pose.position.x = 2.0;
  obj2.kinematics.pose_with_covariance.pose.position.y = 3.0;
  objects_msg.objects.push_back(obj2);
  objects_msg.header.frame_id = kTrackerFrame;  // Set to the same frame as the tracker.

  tracking::TrackedObjects tracks{tracked_object_vec, kTrackerFrame};
  const auto associations = m_associator.assign(objects_msg, tracks);
  ASSERT_EQ(associations.size(), 2UL);
  EXPECT_EQ(associations[0U].matched, tracking::Matched::kNothing);
  EXPECT_EQ(associations[1U].matched, tracking::Matched::kExistingTrack);
  EXPECT_EQ(associations[1U].match_index, 0U);
}

// 10 tracks, 5 detections. Make sure 5 tracks are unassigned
TEST_F(AssociationTester, MoreTracksLessObjects)
{
  const auto num_tracks = 10U;
  auto num_associated_dets = 0U;

  TrackedObjects tracks_msg;
  std::vector<tracking::TrackedObject> tracked_object_vec{};
  DetectedObjects detections_msg;

  for (size_t i = 0U; i < num_tracks; ++i) {
    DetectedObject current_track;
    const auto current_shape = create_square(4.0F);
    current_track.shape = current_shape;
    current_track.kinematics.pose_with_covariance.pose.position.x = 2.0 *
      static_cast<double>(i + 1U);
    current_track.kinematics.pose_with_covariance.pose.position.y = 2.0 *
      static_cast<double>(i + 1U);
    current_track.kinematics.pose_with_covariance.covariance = m_some_covariance;
    current_track.kinematics.has_position_covariance = true;
    tracked_object_vec.emplace_back(current_track, 0.0, 0.0);

    //  Create detections that can be associated with tracks
    if (i % 2 == 0) {
      ++num_associated_dets;
      DetectedObject current_detection;
      current_detection.shape = current_shape;
      // Move detections a bit to test out distance calculation logic as well
      current_detection.kinematics.pose_with_covariance.pose.position.x =
        current_track.kinematics.pose_with_covariance.pose.position.x + 0.6;
      current_detection.kinematics.pose_with_covariance.pose.position.y =
        current_track.kinematics.pose_with_covariance.pose.position.y + 0.8;
      current_detection.kinematics.pose_with_covariance.covariance = m_some_covariance;

      detections_msg.objects.push_back(current_detection);
    }
  }
  detections_msg.header.frame_id = kTrackerFrame;  // Set to the same frame as the tracker.

  tracking::TrackedObjects tracks{tracked_object_vec, kTrackerFrame};
  const auto associations = m_associator.assign(detections_msg, tracks);

  EXPECT_EQ(m_associator.track_associations().size(), num_tracks);
  const auto & track_associations = m_associator.track_associations();
  const auto unassigned_track_count = std::count_if(
    track_associations.begin(), track_associations.end(),
    [](const auto association) {
      return association.matched == tracking::Matched::kNothing;
    });
  EXPECT_EQ(unassigned_track_count, num_tracks - num_associated_dets);
  ASSERT_EQ(associations.size(), 5UL);
  for (const auto & association : associations) {
    EXPECT_EQ(association.matched, tracking::Matched::kExistingTrack);
    EXPECT_EQ(association.match_index % 2, 0UL);
  }
}
