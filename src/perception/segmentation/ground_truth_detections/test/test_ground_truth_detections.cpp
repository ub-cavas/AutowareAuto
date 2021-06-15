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

#include <fake_test_node/fake_test_node.hpp>

#include "gtest/gtest.h"

#include <memory>

namespace
{

using autoware::ground_truth_detections::GroundTruthDetectionsNode;
using lgsvl_msgs::msg::Detection2DArray;

using FakeNodeFixture = autoware::tools::testing::FakeTestNode;

Detection2DArray make_sample_detections()
{
  Detection2DArray detections;

  return detections;
}

TEST_F(FakeNodeFixture, receive_detections) {
  rclcpp::NodeOptions options{};
  const auto node = std::make_shared<GroundTruthDetectionsNode>();

  Detection2DArray detections = make_sample_detections();

}

TEST(test_ground_truth_detections, expected_failure) {
  EXPECT_TRUE(false);
}


}  // namespace
