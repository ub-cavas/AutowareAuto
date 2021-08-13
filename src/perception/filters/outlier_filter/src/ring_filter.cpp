// Copyright 2021 Tier IV, Inc.
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

#include <algorithm>
#include <vector>

#include "common/types.hpp"
#include "outlier_filter/ring_filter.hpp"

#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/segment_differences.h"

namespace autoware
{
namespace perception
{
namespace filters
{
namespace outlier_filter
{
namespace ring_filter
{

RingFilter::RingFilter(
  common::types::float64_t distance_ratio,
  common::types::float64_t object_length_threshold, int num_points_threshold)
: distance_ratio_(distance_ratio), object_length_threshold_(object_length_threshold),
  num_points_threshold_(num_points_threshold)
{

}

void RingFilter::filter(
  const pcl::PointCloud<common::types::PointXYZIF> & input,
  pcl::PointCloud<pcl::PointXYZ> & output)
{
  std::vector<pcl::PointCloud<common::types::PointXYZIF>> pcl_input_ring_array;
  pcl_input_ring_array.resize(128);  // TODO(j.eccleston): make parameter
  for (const auto & p : input.points) {
    // ID field contains the ring information
    pcl_input_ring_array.at(p.id).push_back(p);
  }

  output.points.reserve(input.points.size());

  // Iterate each ring to filter
  pcl::PointCloud<pcl::PointXYZ> pcl_tmp;
  pcl::PointXYZ p;
  for (const auto & ring_points : pcl_input_ring_array) {
    // If points are below minimal threshold ignore and don't filter
    if (ring_points.points.size() < 2) { // TODO(j.eccleston): make parameter
      continue;
    }

    for (auto iter = std::begin(ring_points.points); iter != std::end(ring_points.points) - 1;
      ++iter)
    {
      p.x = (iter)->x;
      p.y = (iter)->y;
      p.z = (iter)->z;
      pcl_tmp.points.push_back(p);
      // Calculate distance
    }
  }
}

bool RingFilter::is_outlier() const
{
  return true;
}

bool RingFilter::is_max_dist_exceeded(pcl::PointXYZ & curr_point, pcl::PointXYZ & next_point) const {
  // TODO: convert to std::hypot(x,y,z) in C++17/Galactic
  const common::types::float64_t curr_distance = std::sqrt(
    curr_point.x * curr_point.x + curr_point.y * curr_point.y + curr_point.z *
    curr_point.z);
  const common::types::float64_t next_distance = std::sqrt(
    next_point.x * next_point.x + next_point.y * next_point.y + next_point.z *
    next_point.z);
  const common::types::float64_t min_dist = std::min(curr_distance, next_distance);
  const common::types::float64_t max_dist = std::max(curr_distance, next_distance);

  return max_dist < min_dist * distance_ratio_;
}

float RingFilter::calc_azimuth_diff(pcl::PointXYZ pt1, pcl::PointXYZ pt2) const {
  // Calculate pt2 and pt3 azimuth in degrees
  float pt1_azimuth = std::atan2(pt1.x, pt1.y) * (180.0f/common::types::PI);
  float pt2_azimuth = std::atan2(pt2.x, pt2.y) * (180.0f/common::types::PI);
  float azimuth_diff = pt1_azimuth - pt2_azimuth;
  azimuth_diff = azimuth_diff < 0.f ? azimuth_diff + 360.f : azimuth_diff;
  return azimuth_diff;
}

}  // namesapce ring_filter
}  // namespace outlier_filter
}  // namespace filters
}  // namespace perception
}  // namespace autoware
