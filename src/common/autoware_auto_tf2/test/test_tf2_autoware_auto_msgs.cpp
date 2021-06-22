// Copyright 2020, The Autoware Foundation.
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
/// \file
/// \brief This file includes common transoform functionaly for autoware_auto_msgs


#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>
#include <rclcpp/clock.hpp>
#include <memory>

#include "autoware_auto_msgs/msg/detected_object.hpp"
#include "autoware_auto_msgs/msg/detected_objects.hpp"

std::unique_ptr<tf2_ros::Buffer> tf_buffer = nullptr;
constexpr double EPS = 1e-3;

geometry_msgs::msg::TransformStamped filled_transfom()
{
  geometry_msgs::msg::TransformStamped t;
  t.transform.translation.x = 10;
  t.transform.translation.y = 20;
  t.transform.translation.z = 30;
  t.transform.rotation.w = 0;
  t.transform.rotation.x = 1;
  t.transform.rotation.y = 0;
  t.transform.rotation.z = 0;
  t.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(2));
  t.header.frame_id = "A";
  t.child_frame_id = "B";

  return t;
}


TEST(Tf2AutowareAuto, DoTransformPoint32)
{
  const auto trans = filled_transfom();
  geometry_msgs::msg::Point32 p1;
  p1.x = 1;
  p1.y = 2;
  p1.z = 3;

  // doTransform
  geometry_msgs::msg::Point32 p_out;
  tf2::doTransform(p1, p_out, trans);

  EXPECT_NEAR(p_out.x, 11, EPS);
  EXPECT_NEAR(p_out.y, 18, EPS);
  EXPECT_NEAR(p_out.z, 27, EPS);
}


TEST(Tf2AutowareAuto, DoTransformPolygon)
{
  const auto trans = filled_transfom();
  geometry_msgs::msg::Polygon poly;
  geometry_msgs::msg::Point32 p1;
  p1.x = 1;
  p1.y = 2;
  p1.z = 3;
  poly.points.push_back(p1);
  // doTransform
  geometry_msgs::msg::Polygon poly_out;
  tf2::doTransform(poly, poly_out, trans);

  EXPECT_NEAR(poly_out.points[0].x, 11, EPS);
  EXPECT_NEAR(poly_out.points[0].y, 18, EPS);
  EXPECT_NEAR(poly_out.points[0].z, 27, EPS);
}


TEST(Tf2AutowareAuto, DoTransformQuaternion32)
{
  const auto trans = filled_transfom();
  autoware_auto_msgs::msg::Quaternion32 q1;
  q1.w = 0;
  q1.x = 0;
  q1.y = 0;
  q1.z = 1;

  // doTransform
  autoware_auto_msgs::msg::Quaternion32 q_out;
  tf2::doTransform(q1, q_out, trans);

  EXPECT_NEAR(q_out.x, 0.0, EPS);
  EXPECT_NEAR(q_out.y, 1.0, EPS);
  EXPECT_NEAR(q_out.z, 0.0, EPS);
  EXPECT_NEAR(q_out.w, 0.0, EPS);
}


TEST(Tf2AutowareAuto, DoTransformDetectedObject)
{
  const auto trans = filled_transfom();
  autoware_auto_msgs::msg::DetectedObject do1;
  do1.kinematics.orientation.w = 0;
  do1.kinematics.orientation.x = 0;
  do1.kinematics.orientation.y = 0;
  do1.kinematics.orientation.z = 1;
  do1.kinematics.centroid_position.x = 1;
  do1.kinematics.centroid_position.y = 2;
  do1.kinematics.centroid_position.z = 3;
  do1.shape.polygon.points.resize(4);
  do1.shape.polygon.points[0].x = 4;
  do1.shape.polygon.points[0].y = 5;
  do1.shape.polygon.points[0].z = 6;
  do1.shape.polygon.points[1].x = 7;
  do1.shape.polygon.points[1].y = 8;
  do1.shape.polygon.points[1].z = 9;
  do1.shape.polygon.points[2].x = 10;
  do1.shape.polygon.points[2].y = 11;
  do1.shape.polygon.points[2].z = 12;
  do1.shape.polygon.points[3].x = 13;
  do1.shape.polygon.points[3].y = 14;
  do1.shape.polygon.points[3].z = 15;

  // doTransform
  autoware_auto_msgs::msg::DetectedObject do_out;
  tf2::doTransform(do1, do_out, trans);

  EXPECT_NEAR(do_out.kinematics.orientation.x, 0.0, EPS);
  EXPECT_NEAR(do_out.kinematics.orientation.y, 1.0, EPS);
  EXPECT_NEAR(do_out.kinematics.orientation.z, 0.0, EPS);
  EXPECT_NEAR(do_out.kinematics.orientation.w, 0.0, EPS);
  EXPECT_NEAR(do_out.kinematics.centroid_position.x, 11, EPS);
  EXPECT_NEAR(do_out.kinematics.centroid_position.y, 18, EPS);
  EXPECT_NEAR(do_out.kinematics.centroid_position.z, 27, EPS);
  EXPECT_NEAR(do_out.shape.polygon.points[0].x, 14, EPS);
  EXPECT_NEAR(do_out.shape.polygon.points[0].y, 15, EPS);
  EXPECT_NEAR(do_out.shape.polygon.points[0].z, 24, EPS);
  EXPECT_NEAR(do_out.shape.polygon.points[1].x, 17, EPS);
  EXPECT_NEAR(do_out.shape.polygon.points[1].y, 12, EPS);
  EXPECT_NEAR(do_out.shape.polygon.points[1].z, 21, EPS);
  EXPECT_NEAR(do_out.shape.polygon.points[2].x, 20, EPS);
  EXPECT_NEAR(do_out.shape.polygon.points[2].y, 9, EPS);
  EXPECT_NEAR(do_out.shape.polygon.points[2].z, 18, EPS);
  EXPECT_NEAR(do_out.shape.polygon.points[3].x, 23, EPS);
  EXPECT_NEAR(do_out.shape.polygon.points[3].y, 6, EPS);
  EXPECT_NEAR(do_out.shape.polygon.points[3].z, 15, EPS);

  // Testing unused fields are unmodified
  EXPECT_EQ(do_out.classification.size(), do1.classification.size());
  EXPECT_EQ(do_out.kinematics.position_covariance, do1.kinematics.position_covariance);
  EXPECT_EQ(do_out.kinematics.position_covariance, do1.kinematics.position_covariance);
  EXPECT_EQ(do_out.kinematics.has_position_covariance, do1.kinematics.has_position_covariance);
  EXPECT_EQ(do_out.kinematics.orientation_availability, do1.kinematics.orientation_availability);
  EXPECT_EQ(do_out.kinematics.twist, do1.kinematics.twist);
  EXPECT_EQ(do_out.kinematics.has_twist, do1.kinematics.has_twist);
  EXPECT_EQ(do_out.kinematics.has_twist_covariance, do1.kinematics.has_twist_covariance);
}

TEST(Tf2AutowareAuto, TransformDetectedObjects)
{
  autoware_auto_msgs::msg::DetectedObject do1;
  do1.kinematics.orientation.w = 0;
  do1.kinematics.orientation.x = 0;
  do1.kinematics.orientation.y = 0;
  do1.kinematics.orientation.z = 1;
  do1.kinematics.centroid_position.x = 20;
  do1.kinematics.centroid_position.y = 21;
  do1.kinematics.centroid_position.z = 22;
  do1.shape.polygon.points.resize(4);
  do1.shape.polygon.points[0].x = 23;
  do1.shape.polygon.points[0].y = 24;
  do1.shape.polygon.points[0].z = 25;
  do1.shape.polygon.points[1].x = 26;
  do1.shape.polygon.points[1].y = 27;
  do1.shape.polygon.points[1].z = 28;
  do1.shape.polygon.points[2].x = 29;
  do1.shape.polygon.points[2].y = 30;
  do1.shape.polygon.points[2].z = 31;
  do1.shape.polygon.points[3].x = 32;
  do1.shape.polygon.points[3].y = 33;
  do1.shape.polygon.points[3].z = 34;

  autoware_auto_msgs::msg::DetectedObject do2;
  do2.kinematics.orientation.w = 0.707f;
  do2.kinematics.orientation.x = -0.706f;
  do2.kinematics.orientation.y = 0;
  do2.kinematics.orientation.z = 0;
  do2.kinematics.centroid_position.x = 50;
  do2.kinematics.centroid_position.y = 51;
  do2.kinematics.centroid_position.z = 52;
  do2.shape.polygon.points.resize(4);
  do2.shape.polygon.points[0].x = 53;
  do2.shape.polygon.points[0].y = 54;
  do2.shape.polygon.points[0].z = 55;
  do2.shape.polygon.points[1].x = 56;
  do2.shape.polygon.points[1].y = 57;
  do2.shape.polygon.points[1].z = 58;
  do2.shape.polygon.points[2].x = 59;
  do2.shape.polygon.points[2].y = 50;
  do2.shape.polygon.points[2].z = 51;
  do2.shape.polygon.points[3].x = 52;
  do2.shape.polygon.points[3].y = 53;
  do2.shape.polygon.points[3].z = 54;

  autoware_auto_msgs::msg::DetectedObjects dos1;
  dos1.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(2));
  dos1.header.frame_id = "A";
  dos1.objects.push_back(do1);
  dos1.objects.push_back(do2);

  // simple api
  const auto dos_simple = tf_buffer->transform(dos1, "B", tf2::durationFromSec(2.0));


  EXPECT_EQ(dos_simple.header.frame_id, "B");

  // checking objects[0]
  EXPECT_NEAR(dos_simple.objects[0].kinematics.orientation.x, 0.0, EPS);
  EXPECT_NEAR(dos_simple.objects[0].kinematics.orientation.y, 1.0, EPS);
  EXPECT_NEAR(dos_simple.objects[0].kinematics.orientation.z, 0.0, EPS);
  EXPECT_NEAR(dos_simple.objects[0].kinematics.orientation.w, 0.0, EPS);
  EXPECT_NEAR(dos_simple.objects[0].kinematics.centroid_position.x, 10, EPS);
  EXPECT_NEAR(dos_simple.objects[0].kinematics.centroid_position.y, -1, EPS);
  EXPECT_NEAR(dos_simple.objects[0].kinematics.centroid_position.z, 8, EPS);
  EXPECT_NEAR(dos_simple.objects[0].shape.polygon.points[0].x, 13, EPS);
  EXPECT_NEAR(dos_simple.objects[0].shape.polygon.points[0].y, -4, EPS);
  EXPECT_NEAR(dos_simple.objects[0].shape.polygon.points[0].z, 5, EPS);
  EXPECT_NEAR(dos_simple.objects[0].shape.polygon.points[1].x, 16, EPS);
  EXPECT_NEAR(dos_simple.objects[0].shape.polygon.points[1].y, -7, EPS);
  EXPECT_NEAR(dos_simple.objects[0].shape.polygon.points[1].z, 2, EPS);
  EXPECT_NEAR(dos_simple.objects[0].shape.polygon.points[2].x, 19, EPS);
  EXPECT_NEAR(dos_simple.objects[0].shape.polygon.points[2].y, -10, EPS);
  EXPECT_NEAR(dos_simple.objects[0].shape.polygon.points[2].z, -1, EPS);
  EXPECT_NEAR(dos_simple.objects[0].shape.polygon.points[3].x, 22, EPS);
  EXPECT_NEAR(dos_simple.objects[0].shape.polygon.points[3].y, -13, EPS);
  EXPECT_NEAR(dos_simple.objects[0].shape.polygon.points[3].z, -4, EPS);

  // checking objects[1]
  EXPECT_NEAR(dos_simple.objects[1].kinematics.orientation.x, 0.707, EPS);
  EXPECT_NEAR(dos_simple.objects[1].kinematics.orientation.y, 0.0, EPS);
  EXPECT_NEAR(dos_simple.objects[1].kinematics.orientation.z, 0.0, EPS);
  EXPECT_NEAR(dos_simple.objects[1].kinematics.orientation.w, 0.707, EPS);
  EXPECT_NEAR(dos_simple.objects[1].kinematics.centroid_position.x, 40, EPS);
  EXPECT_NEAR(dos_simple.objects[1].kinematics.centroid_position.y, -31, EPS);
  EXPECT_NEAR(dos_simple.objects[1].kinematics.centroid_position.z, -22, EPS);
  EXPECT_NEAR(dos_simple.objects[1].shape.polygon.points[0].x, 43, EPS);
  EXPECT_NEAR(dos_simple.objects[1].shape.polygon.points[0].y, -34, EPS);
  EXPECT_NEAR(dos_simple.objects[1].shape.polygon.points[0].z, -25, EPS);
  EXPECT_NEAR(dos_simple.objects[1].shape.polygon.points[1].x, 46, EPS);
  EXPECT_NEAR(dos_simple.objects[1].shape.polygon.points[1].y, -37, EPS);
  EXPECT_NEAR(dos_simple.objects[1].shape.polygon.points[1].z, -28, EPS);
  EXPECT_NEAR(dos_simple.objects[1].shape.polygon.points[2].x, 49, EPS);
  EXPECT_NEAR(dos_simple.objects[1].shape.polygon.points[2].y, -30, EPS);
  EXPECT_NEAR(dos_simple.objects[1].shape.polygon.points[2].z, -21, EPS);
  EXPECT_NEAR(dos_simple.objects[1].shape.polygon.points[3].x, 42, EPS);
  EXPECT_NEAR(dos_simple.objects[1].shape.polygon.points[3].y, -33, EPS);
  EXPECT_NEAR(dos_simple.objects[1].shape.polygon.points[3].z, -24, EPS);


  // advanced api
  const auto dos_advanced = tf_buffer->transform(
    dos1, "B",
    tf2::timeFromSec(2.0), "A", tf2::durationFromSec(3.0));

  EXPECT_EQ(dos_advanced.header.frame_id, "B");

  // checking objects[0]
  EXPECT_NEAR(dos_advanced.objects[0].kinematics.orientation.x, 0.0, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].kinematics.orientation.y, 1.0, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].kinematics.orientation.z, 0.0, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].kinematics.orientation.w, 0.0, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].kinematics.centroid_position.x, 10, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].kinematics.centroid_position.y, -1, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].kinematics.centroid_position.z, 8, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].shape.polygon.points[0].x, 13, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].shape.polygon.points[0].y, -4, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].shape.polygon.points[0].z, 5, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].shape.polygon.points[1].x, 16, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].shape.polygon.points[1].y, -7, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].shape.polygon.points[1].z, 2, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].shape.polygon.points[2].x, 19, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].shape.polygon.points[2].y, -10, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].shape.polygon.points[2].z, -1, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].shape.polygon.points[3].x, 22, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].shape.polygon.points[3].y, -13, EPS);
  EXPECT_NEAR(dos_advanced.objects[0].shape.polygon.points[3].z, -4, EPS);

  // checking objects[1]
  EXPECT_NEAR(dos_advanced.objects[1].kinematics.orientation.x, 0.707, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].kinematics.orientation.y, 0.0, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].kinematics.orientation.z, 0.0, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].kinematics.orientation.w, 0.707, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].kinematics.centroid_position.x, 40, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].kinematics.centroid_position.y, -31, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].kinematics.centroid_position.z, -22, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].shape.polygon.points[0].x, 43, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].shape.polygon.points[0].y, -34, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].shape.polygon.points[0].z, -25, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].shape.polygon.points[1].x, 46, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].shape.polygon.points[1].y, -37, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].shape.polygon.points[1].z, -28, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].shape.polygon.points[2].x, 49, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].shape.polygon.points[2].y, -30, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].shape.polygon.points[2].z, -21, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].shape.polygon.points[3].x, 42, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].shape.polygon.points[3].y, -33, EPS);
  EXPECT_NEAR(dos_advanced.objects[1].shape.polygon.points[3].z, -24, EPS);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf_buffer = std::make_unique<tf2_ros::Buffer>(clock);
  tf_buffer->setUsingDedicatedThread(true);

  // populate buffer
  const geometry_msgs::msg::TransformStamped t = filled_transfom();
  tf_buffer->setTransform(t, "test");

  int ret = RUN_ALL_TESTS();
  return ret;
}
