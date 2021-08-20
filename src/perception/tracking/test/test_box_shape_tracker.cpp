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


#include <autoware_auto_msgs/msg/detected_object.hpp>
#include <tracking/box_shape_tracker.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <vector>

using autoware::perception::tracking::BoxShapeTracker;
using autoware_auto_msgs::msg::DetectedObject;
using autoware_auto_msgs::msg::ObjectClassification;
using autoware::common::types::float32_t;

namespace
{
geometry_msgs::msg::Point32 make_point(float x, float y, float z)
{
  geometry_msgs::msg::Point32 point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}
}  // namespace


TEST(BoxShapeTrackerTest, TestInit) {
  BoxShapeTracker tracker;
  const auto shape = tracker.current_shape();
  EXPECT_EQ(shape.polygon.points.size(), 4UL);
  EXPECT_FLOAT_EQ(shape.height, 0.0F);
  for (const auto & point : shape.polygon.points) {
    EXPECT_FLOAT_EQ(point.x, 0.0F);
    EXPECT_FLOAT_EQ(point.y, 0.0F);
    EXPECT_FLOAT_EQ(point.z, 0.0F);
  }
}

TEST(BoxShapeTrackerTest, UpdateOnce) {
  BoxShapeTracker tracker;
  autoware_auto_msgs::msg::Shape input_shape;
  input_shape.height = 10.0F;
  input_shape.polygon.points.emplace_back(make_point(1.0F, 1.0F, 1.0F));
  input_shape.polygon.points.emplace_back(make_point(2.0F, 2.0F, 2.0F));
  input_shape.polygon.points.emplace_back(make_point(3.0F, 3.0F, 3.0F));
  input_shape.polygon.points.emplace_back(make_point(4.0F, 4.0F, 4.0F));
  tracker.update(input_shape, 0.1F);
  const auto shape = tracker.current_shape();
  EXPECT_EQ(shape.polygon.points.size(), input_shape.polygon.points.size());
  EXPECT_FLOAT_EQ(shape.height, input_shape.height);
  for (auto i = 0U; i < shape.polygon.points.size(); ++i) {
    const auto input_point = input_shape.polygon.points[i];
    const auto result_point = shape.polygon.points[i];
    EXPECT_FLOAT_EQ(result_point.x, input_point.x);
    EXPECT_FLOAT_EQ(result_point.y, input_point.y);
    EXPECT_FLOAT_EQ(result_point.z, input_point.z);
  }
}

TEST(BoxShapeTrackerTest, UpdateWrong) {
  BoxShapeTracker tracker;
  autoware_auto_msgs::msg::Shape input_shape;
  input_shape.height = 10.0F;
  input_shape.polygon.points.emplace_back(make_point(1.0F, 1.0F, 1.0F));
  // Not enough points in shape.
  EXPECT_THROW(tracker.update(input_shape, 0.1F), std::domain_error);
}
