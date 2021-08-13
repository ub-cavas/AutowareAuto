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
  common::types::float32_t object_length_threshold, int num_points_threshold)
: distance_ratio_(distance_ratio), object_length_threshold_(object_length_threshold),
  num_points_threshold_(num_points_threshold)
{

}

void RingFilter::filter(
  const pcl::PointCloud<common::types::PointXYZIF> & input,
  pcl::PointCloud<pcl::PointXYZ> & output)
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>> pcl_input_ring_array;
  pcl_input_ring_array.resize(128);  // TODO(j.eccleston): make parameter
  pcl::PointXYZ tmp_p;
  for (const auto & p : input.points) {
    // ID field contains the ring information
    tmp_p.x = p.x;
    tmp_p.y = p.y;
    tmp_p.z = p.z;
    pcl_input_ring_array.at(p.id).push_back(tmp_p);
  }

  output.points.reserve(input.points.size());

  // Iterate each ring to filter
  pcl::PointCloud<pcl::PointXYZ> pcl_tmp;
  for (const auto & ring_points : pcl_input_ring_array) {
    // If points are below minimal threshold ignore and don't filter
    if (ring_points.points.size() < 2) { // TODO(j.eccleston): make parameter
      continue;
    }

    for (auto iter = std::begin(ring_points.points); iter != std::end(ring_points.points) - 1;
      ++iter)
    {
      pcl_tmp.points.push_back(*iter);

      if (!is_outlier(*iter, *(iter + 1))) {

      }
    }
  }
}

bool RingFilter::is_outlier(const pcl::PointXYZ & pt1, const pcl::PointXYZ & pt2) const
{
  // TODO(j.eccleston): Should parameterise the 100.0f
  return is_max_dist_exceeded(pt1, pt2) && calc_azimuth_diff(pt1, pt2) < 100.0f;
}

bool RingFilter::is_max_dist_exceeded(const pcl::PointXYZ & pt1, const pcl::PointXYZ & pt2) const
{
  // TODO: convert to std::hypot(x,y,z) in C++17/Galactic
  const common::types::float64_t curr_distance = std::sqrt(
    pt1.x * pt1.x + pt1.y * pt1.y + pt1.z * pt1.z);
  const common::types::float64_t next_distance = std::sqrt(
    pt2.x * pt2.x + pt2.y * pt2.y + pt2.z * pt2.z);
  const common::types::float64_t min_dist = std::min(curr_distance, next_distance);
  const common::types::float64_t max_dist = std::max(curr_distance, next_distance);

  return max_dist < min_dist * distance_ratio_;
}

float RingFilter::calc_azimuth_diff(const pcl::PointXYZ & pt1, const pcl::PointXYZ & pt2) const
{
  // Calculate pt2 and pt3 azimuth in degrees
  float pt1_azimuth = std::atan2(pt1.x, pt1.y) * (180.0f / common::types::PI);
  float pt2_azimuth = std::atan2(pt2.x, pt2.y) * (180.0f / common::types::PI);
  float azimuth_diff = pt1_azimuth - pt2_azimuth;
  azimuth_diff = azimuth_diff < 0.f ? azimuth_diff + 360.f : azimuth_diff;
  return azimuth_diff;
}

bool RingFilter::is_object_threshold_exceeded(const pcl::PointXYZ & pt1, const pcl::PointXYZ & pt2) const
{
  return (pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y) + (pt1.z - pt2.z) *
         (pt1.z - pt2.z) >= object_length_threshold_ * object_length_threshold_;
}

}  // namesapce ring_filter
}  // namespace outlier_filter
}  // namespace filters
}  // namespace perception
}  // namespace autoware
