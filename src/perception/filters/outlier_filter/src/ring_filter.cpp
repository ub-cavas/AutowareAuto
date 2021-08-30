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
#include "point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

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
  common::types::float32_t object_length_threshold, std::size_t num_points_threshold)
: distance_ratio_(distance_ratio), object_length_threshold_(object_length_threshold),
  num_points_threshold_(num_points_threshold)
{}

void RingFilter::filter(
  const sensor_msgs::msg::PointCloud2 & input,
  sensor_msgs::msg::PointCloud2 & output)
{
  // Declare the pointcloud msg wrapper view here
  // common::types::PointXYZIF
  point_cloud_msg_wrapper::PointCloud2View<autoware::common::types::PointXYZIF> input_view{input};
  point_cloud_msg_wrapper::PointCloud2Modifier<autoware::common::types::PointXYZI> output_modifier{output};

  std::vector<std::vector<autoware::common::types::PointXYZI>> input_ring_array;
  input_ring_array.resize(128);  // TODO(j.eccleston): make parameter
  for (const auto & p : input_view) {
    input_ring_array.at(p.id).push_back({p.x, p.y, p.z, p.intensity});
  }

  output_modifier.reserve(input_view.size());

  // // Iterate each ring to filter
  std::vector<autoware::common::types::PointXYZI> tmp_pointcloud_array;
  for (const auto & ring_points : input_ring_array) {
    // If points are below minimal threshold ignore and don't filter
    if (ring_points.size() < 2) {  // TODO(j.eccleston): make parameter
      continue;
    }

    // TODO(j.eccleston): there must be a way to simplify this logic... seems unnecessary (what are we really trying to achieve here)
    for (auto iter = ring_points.begin(); iter != ring_points.end() - 1;
      ++iter)
    {
      tmp_pointcloud_array.push_back(*iter);
      if (!is_outlier(*iter, *(iter + 1))) {
        if (tmp_pointcloud_array.size() > num_points_threshold_ ||
          is_object_threshold_exceeded(tmp_pointcloud_array.front(), tmp_pointcloud_array.back()))
        {
          for (const auto & p : tmp_pointcloud_array) {
            output_modifier.push_back(p);
          }
        }
        tmp_pointcloud_array.clear();
      }
    }

    if (tmp_pointcloud_array.size() > num_points_threshold_ ||
      is_object_threshold_exceeded(tmp_pointcloud_array.front(), tmp_pointcloud_array.back()))
    {
      for (const auto & p : tmp_pointcloud_array) {
        output_modifier.push_back(p);
      }
    }
    tmp_pointcloud_array.clear();
  }
}

common::types::bool8_t RingFilter::is_outlier(
  const autoware::common::types::PointXYZI & pt1,
  const autoware::common::types::PointXYZI & pt2) const
{
  // TODO(j.eccleston): Should parameterise the 100.0f
  return is_max_dist_exceeded(pt1, pt2) && calc_azimuth_diff(pt1, pt2) < 100.0f;
}

common::types::bool8_t RingFilter::is_max_dist_exceeded(
  const autoware::common::types::PointXYZI & pt1,
  const autoware::common::types::PointXYZI & pt2) const
{
  // TODO(j.eccleston): convert to std::hypot(x,y,z) in C++17/Galactic
  const common::types::float64_t curr_distance = std::sqrt(
    pt1.x * pt1.x + pt1.y * pt1.y + pt1.z * pt1.z);
  const common::types::float64_t next_distance = std::sqrt(
    pt2.x * pt2.x + pt2.y * pt2.y + pt2.z * pt2.z);
  const common::types::float64_t min_dist = std::min(curr_distance, next_distance);
  const common::types::float64_t max_dist = std::max(curr_distance, next_distance);

  return max_dist < min_dist * distance_ratio_;
}

common::types::float32_t RingFilter::calc_azimuth_diff(
  const autoware::common::types::PointXYZI & pt1,
  const autoware::common::types::PointXYZI & pt2) const
{
  // Calculate pt2 and pt3 azimuth in degrees
  common::types::float32_t pt1_azimuth = std::atan2(pt1.x, pt1.y) * (180.0f / common::types::PI);
  common::types::float32_t pt2_azimuth = std::atan2(pt2.x, pt2.y) * (180.0f / common::types::PI);
  common::types::float32_t azimuth_diff = pt1_azimuth - pt2_azimuth;
  azimuth_diff = azimuth_diff < 0.f ? azimuth_diff + 360.f : azimuth_diff;
  return azimuth_diff;
}

common::types::bool8_t RingFilter::is_object_threshold_exceeded(
  const autoware::common::types::PointXYZI & pt1,
  const autoware::common::types::PointXYZI & pt2) const
{
  return (pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y) + (pt1.z - pt2.z) *
         (pt1.z - pt2.z) >= object_length_threshold_ * object_length_threshold_;
}

}  // namespace ring_filter
}  // namespace outlier_filter
}  // namespace filters
}  // namespace perception
}  // namespace autoware
