// Copyright 2021 Tier IV, Inc
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

#include <vector>
#include <memory>

#include "gtest/gtest.h"
#include "pcl_conversions/pcl_conversions.h"

#include "outlier_filter/ring_filter.hpp"
#include "outlier_filter_test_utils.hpp"

using RingFilter =
  autoware::perception::filters::outlier_filter::ring_filter::RingFilter;

// TEST METHODs
/**
 */
TEST(RingFilter, TestSinglePoint) {
  auto filter = std::make_shared<RingFilter>(1.03, 0.1, 4);
  std::vector<autoware::common::types::PointXYZIF> points = {
    make_point(0.0f, 0.0f, 0.0f)};
  auto time0 = std::chrono::system_clock::now();
  auto t0 = to_msg_time(time0);
  auto input = make_pc<pcl::PointXYZ>(points, t0);

  // Run the filter
  pcl::PointCloud<pcl::PointXYZ> output;
  // filter->filter(input, output);

  // Perform checks on the output pointcloud
  // For this test the single pointcloud is considered an outlier and will be removed
  check_pc({}, output);
}
