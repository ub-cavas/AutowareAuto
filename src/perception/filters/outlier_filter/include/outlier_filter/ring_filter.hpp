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

/// \copyright Copyright 2021 Tier IV, Inc.
/// \file
/// \brief This file defines the RingFilter class.

#ifndef OUTLIER_FILTER__RING_FILTER_HPP_
#define OUTLIER_FILTER__RING_FILTER_HPP_

#include <vector>

#include "common/types.hpp"
#include "outlier_filter/visibility_control.hpp"

#include "pcl/filters/voxel_grid.h"
#include "pcl/search/pcl_search.h"

namespace autoware
{
namespace perception
{
namespace filters
{
namespace outlier_filter
{
/** \brief Namespace for the RingFilter library */
namespace ring_filter
{

/** \class RingFilter
 * \brief Library for using a ring based filtering algorithm to perform outlier filtering
 */
class OUTLIER_FILTER_PUBLIC RingFilter
{
public:
  /** \brief Constructor for the RingFilter class
   * \param distance_ratio
   * \param object_length_threshold
   * \param num_points_threshold
   */
  OUTLIER_FILTER_PUBLIC RingFilter(
    common::types::float64_t distance_ratio,
    common::types::float32_t object_length_threshold,
    int num_points_threshold);

  /** \brief Filter function that runs the ring based algorithm.
   * \param input The input point cloud for filtering
   * \param output The output point cloud
   */
  void OUTLIER_FILTER_PUBLIC filter(
    const pcl::PointCloud<common::types::PointXYZIF> & input,
    pcl::PointCloud<pcl::PointXYZ> & output);

  /** \brief Update dynamically configurable parameters
   * \param distance_ratio Parameter that updates the distance_ratio_ member variable
   * \param object_length_threshold Parameter that updates the object_length_threshold_ member variable
   * \param num_points_threshold Parameter that updates the num_points_threshold_ member variable
   */
  void OUTLIER_FILTER_PUBLIC update_parameters(
    common::types::float64_t distance_ratio,
    common::types::float32_t object_length_threshold,
    int num_points_threshold)
  {
    distance_ratio_ = distance_ratio;
    object_length_threshold_ = object_length_threshold;
    num_points_threshold_ = num_points_threshold;
  }

private:
  /** \brief */
  common::types::float64_t distance_ratio_;

  /** \brief */
  common::types::float32_t object_length_threshold_;

  /** \brief */
  int num_points_threshold_;

  /** \brief
   */
  bool is_outlier(const pcl::PointXYZ & pt1, const pcl::PointXYZ & pt2) const;

  bool is_max_dist_exceeded(const pcl::PointXYZ & pt1, const pcl::PointXYZ & pt2) const;

  float calc_azimuth_diff(const pcl::PointXYZ & pt1, const pcl::PointXYZ & pt2) const;

  bool is_object_threshold_exceeded(const pcl::PointXYZ & pt1, const pcl::PointXYZ & pt2) const;
};
}  // namesapce ring_filter
}  // namespace outlier_filter
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // OUTLIER_FILTER__RING_FILTER_HPP_
