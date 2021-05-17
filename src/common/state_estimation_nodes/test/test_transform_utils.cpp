// Copyright 2021 Apex.AI, Inc.
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

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#include <gtest/gtest.h>


#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <state_estimation_nodes/transform_utils.hpp>

#include <tf2/buffer_core.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Geometry>

namespace
{
const auto kEpsilon = 1e-10F;
}  // namespace

/// @test Test that a frame can be switched for the geometry_msgs::msg::PoseWithCovarianceStamped.
///
/// @details This test tests the following configuration:
///                              ▲
///                              │
///                              │
/// ▲                            │
/// │                            │
/// │                 ◄──────────┘
/// │                   old_parent
/// │
/// │
/// └───────────►
///  new_parent
///
TEST(TransformUtilsTest, SwitchFramesPoseWithCovariance) {
  Eigen::Isometry3d tf__new_parent__old_parent{Eigen::Isometry3d::Identity()};
  Eigen::Quaterniond rotation;
  rotation = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
  tf__new_parent__old_parent.linear() = rotation.toRotationMatrix();
  tf__new_parent__old_parent.translation() = Eigen::Vector3d{42.0, 23.0, 0.0};

  tf2::BufferCore buffer;
  geometry_msgs::msg::TransformStamped transform;
  transform.set__child_frame_id("old_parent");
  transform.header.set__frame_id("new_parent");
  transform.header.stamp.set__sec(42).set__nanosec(42);
  transform.transform = tf2::eigenToTransform(tf__new_parent__old_parent).transform;
  buffer.setTransform(transform, "random_authority");


  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.set__frame_id("old_parent");
  msg.header.stamp.set__sec(42).set__nanosec(42);
  msg.pose.pose = tf2::toMsg(Eigen::Isometry3d::Identity());
  msg.pose.covariance[0] = 42.0;  // x covariance
  msg.pose.covariance[7] = 23.0;  // y covariance
  msg.pose.covariance[14] = 1.0;  // z covariance
  msg.pose.covariance[21] = 42.0;  // roll covariance
  msg.pose.covariance[28] = 23.0;  // pitch covariance
  msg.pose.covariance[35] = 1.0;  // pitch covariance


  const auto new_msg = autoware::common::state_estimation::switch_frames(msg, "new_parent", buffer);
  EXPECT_EQ(new_msg.header.frame_id, "new_parent");
  EXPECT_DOUBLE_EQ(new_msg.pose.pose.position.x, 42.0);
  EXPECT_DOUBLE_EQ(new_msg.pose.pose.position.y, 23.0);
  EXPECT_DOUBLE_EQ(new_msg.pose.pose.position.z, 0.0);
  EXPECT_NEAR(new_msg.pose.pose.orientation.x, rotation.x(), kEpsilon);
  EXPECT_NEAR(new_msg.pose.pose.orientation.y, rotation.y(), kEpsilon);
  EXPECT_NEAR(new_msg.pose.pose.orientation.z, rotation.z(), kEpsilon);
  EXPECT_NEAR(new_msg.pose.pose.orientation.w, rotation.w(), kEpsilon);
  EXPECT_NEAR(new_msg.pose.covariance[0], 23.0, kEpsilon);  // x covariance rotated by 90 deg
  EXPECT_NEAR(new_msg.pose.covariance[7], 42.0, kEpsilon);  // y covariance rotated by 90 deg
  EXPECT_NEAR(new_msg.pose.covariance[14], 1.0, kEpsilon);  // z covariance unchanged
  EXPECT_NEAR(new_msg.pose.covariance[21], 23.0, kEpsilon);  // roll covariance rotated by 90 deg
  EXPECT_NEAR(new_msg.pose.covariance[28], 42.0, kEpsilon);  // pitch covariance rotated by 90 deg
  EXPECT_NEAR(new_msg.pose.covariance[35], 1.0, kEpsilon);  // pitch covariance unchanged
}
