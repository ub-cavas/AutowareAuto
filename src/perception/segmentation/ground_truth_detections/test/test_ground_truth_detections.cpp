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

#include "ground_truth_detections/ground_truth_detections_node.hpp"

#include <geometry_msgs/msg/vector3.hpp>

#include <fake_test_node/fake_test_node.hpp>

#include "gtest/gtest.h"

#include <memory>

namespace
{

using autoware::ground_truth_detections::GroundTruthDetectionsNode;
using autoware_auto_msgs::msg::ClassifiedRoiArray;
using geometry_msgs::msg::Vector3;
using lgsvl_msgs::msg::Detection2DArray;
using lgsvl_msgs::msg::Detection2D;

using FakeNodeFixture = autoware::tools::testing::FakeTestNode;

using namespace std::chrono_literals;

// 3 detections, the latter two overlap
Detection2DArray make_sample_detections()
{
  Detection2DArray detections;
  detections.header.stamp.sec = 34;
  detections.header.stamp.nanosec = 8723U;

  Detection2D d;

  {
    d.label = "car";
    d.header = detections.header;
    // TODO what are the units? Pixel coordinates? WHere is the origin?
    d.bbox.x = 15.3F;
    d.bbox.y = 17.4F;
    d.bbox.height = 5.2F;
    d.bbox.width = 2.7F;
    d.id = 14;
    d.score = 1.0F;
    d.velocity.linear = geometry_msgs::build<Vector3>().x(1.1).y(2.2).z(3.3);
    detections.detections.emplace_back(d);
  }

  {
    d.label = "pedestrian";
    d.bbox.x *= 30.0F;
    d.bbox.y *= 30.0F;
    d.bbox.height *= 0.1;
    d.bbox.width *= 0.1;
    d.id = 28;
    detections.detections.emplace_back(d);
  }

  {
    d.label = "foo";
    d.id = 92;
    detections.detections.emplace_back(d);
  }

  return detections;
}

TEST_F(FakeNodeFixture, receive_detections) {
  rclcpp::NodeOptions options{};
  const auto node = std::make_shared<GroundTruthDetectionsNode>(options);

  ClassifiedRoiArray::SharedPtr last_received_msg{};
  const auto input_topic = "/simulator/ground_truth/detections2D";
  const auto output_topic = "/simulator/ground_truth/detections2D_roi";
  // TODO build error
// /home/frederik.beaujean/AutowareAuto/install/fake_test_node/include/fake_test_node/fake_test_node.hpp: In instantiation of ‘typename rclcpp::Publisher<MsgT>::SharedPtr autoware::tools::testing::detail::FakeNodeCore::create_publisher(const string&, const milliseconds&, int32_t) [with MsgT = lgsvl_msgs::msg::Detection2DArray_<std::allocator<void> >; typename rclcpp::Publisher<MsgT>::SharedPtr = std::shared_ptr<rclcpp::Publisher<lgsvl_msgs::msg::Detection2DArray_<std::allocator<void> >, std::allocator<void> > >; std::string = std::__cxx11::basic_string<char>; std::chrono::milliseconds = std::chrono::duration<long int, std::ratio<1, 1000> >; int32_t = int]’:
///home/frederik.beaujean/AutowareAuto/src/perception/segmentation/ground_truth_detections/test/test_ground_truth_detections.cpp:46:71:   required from here
///home/frederik.beaujean/AutowareAuto/install/fake_test_node/include/fake_test_node/fake_test_node.hpp:104:50: error: conversion to ‘size_t’ {aka ‘long unsigned int’} from ‘int32_t’ {aka ‘int’} may change the sign of the  [-Werror=sign-conversion]
//  104 |       m_fake_node->create_publisher<MsgT>(topic, history_size);
//      |                                                  ^~~~~~~~~~~~
//  auto fake_publisher = create_publisher<Detection2DArray>(input_topic);

#if 1
  auto fake_publisher = create_publisher<Detection2DArray>(input_topic, 1s);

  // TODO correct Igor's sample: create_result_subscription doesn't exist
  auto result_subscription = create_subscription<ClassifiedRoiArray>(
    output_topic, *node,                                                                  //
    [&last_received_msg](
      const ClassifiedRoiArray::SharedPtr msg) {
      last_received_msg = msg;
    });


  Detection2DArray input_msg = make_sample_detections();

  const auto dt{100ms};
  // TODO Test continues after timeout
  // /home/frederik.beaujean/AutowareAuto/install/fake_test_node/include/fake_test_node/fake_test_node.hpp:110: Failure
//Expected: (spent_time) < (timeout), actual: 8-byte object <34-3A 00-00 00-00 00-00> vs 8-byte object <E8-03 00-00 00-00 00-00>
//Nobody is listening to the mock topic we publish.
///home/frederik.beaujean/AutowareAuto/install/fake_test_node/include/fake_test_node/fake_test_node.hpp:110: Failure
//Expected: (spent_time) < (timeout), actual: 8-byte object <98-3A 00-00 00-00 00-00> vs 8-byte object <E8-03 00-00 00-00 00-00>
//Nobody is listening to the mock topic we publish.
  const auto max_wait_time{std::chrono::seconds{1LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (!last_received_msg) {
    fake_publisher->publish(input_msg);
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      FAIL() << "Did not receive a message soon enough.";
    }
  }

  ASSERT_TRUE(last_received_msg);
  ASSERT_EQ(last_received_msg->rois.size(), 3);

  const auto & car_roi = last_received_msg->rois.front();

  ASSERT_EQ(car_roi.classifications.size(), 1);
  EXPECT_FLOAT_EQ(car_roi.classifications.front().probability, 1.0);
  EXPECT_EQ(
    car_roi.classifications.front().classification,
    autoware_auto_msgs::msg::ObjectClassification::CAR);

  ASSERT_EQ(car_roi.polygon.points.size(), 4);

  // lower left corner
  EXPECT_FLOAT_EQ(car_roi.polygon.points[0].x, 15.3F - 0.5F * 2.7F);
  EXPECT_FLOAT_EQ(car_roi.polygon.points[0].y, 15.3F - 0.5F * 5.2F);
  EXPECT_FLOAT_EQ(car_roi.polygon.points[0].z, 0.0F);

  // lower right corner
  EXPECT_FLOAT_EQ(car_roi.polygon.points[0].x, 15.3F + 0.5F * 2.7F);
  EXPECT_FLOAT_EQ(car_roi.polygon.points[0].y, 15.3F - 0.5F * 5.2F);
  EXPECT_FLOAT_EQ(car_roi.polygon.points[0].z, 0.0F);

  // upper right corner
  EXPECT_FLOAT_EQ(car_roi.polygon.points[2].x, 15.3F + 0.5F * 2.7F);
  EXPECT_FLOAT_EQ(car_roi.polygon.points[2].y, 15.3F + 0.5F * 5.2F);
  EXPECT_FLOAT_EQ(car_roi.polygon.points[2].z, 0.0F);

  // upper left corner
  EXPECT_FLOAT_EQ(car_roi.polygon.points[2].x, 15.3F - 0.5F * 2.7F);
  EXPECT_FLOAT_EQ(car_roi.polygon.points[2].y, 15.3F + 0.5F * 5.2F);
  EXPECT_FLOAT_EQ(car_roi.polygon.points[2].z, 0.0F);

  const auto & pedestrian_roi = last_received_msg->rois[1];
  EXPECT_EQ(
    pedestrian_roi.classifications.front().classification,
    autoware_auto_msgs::msg::ObjectClassification::PEDESTRIAN);
  EXPECT_NE(pedestrian_roi.polygon, car_roi.polygon);

  const auto & unknown_roi = last_received_msg->rois.back();
  EXPECT_EQ(
    unknown_roi.classifications.front().classification,
    autoware_auto_msgs::msg::ObjectClassification::UNKNOWN);
  EXPECT_EQ(unknown_roi.polygon, pedestrian_roi.polygon);
#endif
}

TEST(test_ground_truth_detections, expected_failure) {
  EXPECT_TRUE(false);
}


}  // namespace
