// Copyright 2017-2019 the Autoware Foundation
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

/// \file
/// \brief Common functionality for bounding box computation algorithms

#ifndef GEOMETRY__BOUNDING_BOX_L_SHAPE_FITTING_HPP_
#define GEOMETRY__BOUNDING_BOX_L_SHAPE_FITTING_HPP_

#include <limits>
#include <utility>
#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>
#include <geometry/bounding_box/eigenbox_2d.hpp>

namespace autoware
{
namespace common
{
namespace geometry
{
namespace bounding_box
{
namespace details
{
float32_t calculate_closeness_criterion(std::vector<float32_t> & C_1, std::vector<float32_t> & C_2)
{
  // Paper : Algo.4 Closeness Criterion
  const float32_t min_c_1 = *std::min_element(C_1.begin(), C_1.end());  // col.2, Algo.4
  const float32_t max_c_1 = *std::max_element(C_1.begin(), C_1.end());  // col.2, Algo.4
  const float32_t min_c_2 = *std::min_element(C_2.begin(), C_2.end());  // col.3, Algo.4
  const float32_t max_c_2 = *std::max_element(C_2.begin(), C_2.end());  // col.3, Algo.4

  std::vector<float32_t> D_1;  // col.4, Algo.4
  for (const auto & c_1_element : C_1) {
    const float32_t v = std::min(max_c_1 - c_1_element, c_1_element - min_c_1);
    D_1.push_back(v * v);
  }

  std::vector<float32_t> D_2;  // col.5, Algo.4
  for (const auto & c_2_element : C_2) {
    const float v = std::min(max_c_2 - c_2_element, c_2_element - min_c_2);
    D_2.push_back(v * v);
  }
  auto d_min = static_cast<float32_t>(0.1 * 0.1);
  auto d_max = static_cast<float32_t>(0.4 * 0.4);
  float32_t beta = 0;  // col.6, Algo.4
  for (size_t i = 0; i < D_1.size(); ++i) {
    if (d_max < std::min(D_1.at(i), D_2.at(i))) {
      continue;
    }
    const float32_t d = std::max(std::min(D_1.at(i), D_2.at(i)), d_min);
    beta += static_cast<float32_t>(1.0 / static_cast<double>(d));
  }
  return beta;
}

template <typename IT>
BoundingBox fit_l_shape_fitting(
  const IT begin, const IT end, const float min_angle, const float max_angle)
{
  const float32_t epsilon = 0.001F;

  using PointT = base_type<decltype(*begin)>;

  using point_adapter::x_;
  using point_adapter::y_;
  using point_adapter::z_;
  using point_adapter::xr_;
  using point_adapter::yr_;

  const auto & pt_first = *begin;
  float32_t min_z = z_(pt_first);
  float32_t max_z = z_(pt_first);

  for (auto it = begin; it != end; ++it) {
    const auto & pt = *it;
    min_z = std::min(pt.z, z_(pt));
    max_z = std::max(pt.z, z_(pt));
  }

  /*
   * Paper : IV2017, Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners
   * Authors : Xio Zhang, Wenda Xu, Chiyu Dong and John M. Dolan
   */

  // Paper : Algo.2 Search-Based Rectangle Fitting
  std::vector<std::pair<float32_t /*theta*/, float32_t /*q*/>> Q;
  auto angle_resolution = static_cast<float32_t>(M_PI / 180.0);
  for (float32_t theta = min_angle; theta <= max_angle + epsilon; theta += angle_resolution) {
    PointT e_1;
    xr_(e_1)=std::cos(theta);
    yr_(e_1)=std::sin(theta);

    PointT e_2;
    xr_(e_2) = -std::sin(theta); // col.4, Algo.2
    yr_(e_2) = std::cos(theta);
    std::vector<float32_t> C_1;                // col.5, Algo.2
    std::vector<float32_t> C_2;                // col.6, Algo.2

    for (auto it = begin; it != end; ++it) {
      const PointT & pt = *it;
      C_1.push_back((x_(pt) * x_(e_1)) + (y_(pt) * y_(e_1)));
      C_2.push_back((x_(pt) * x_(e_2)) + (y_(pt) * y_(e_2)));
    }

    float32_t q = autoware::common::geometry::bounding_box::details::calculate_closeness_criterion(
      C_1, C_2);                            // col.7, Algo.2
    Q.push_back(std::make_pair(theta, q));  // col.8, Algo.2
  }

  float32_t theta_star{0.0};  // col.10, Algo.2
  float32_t max_q = 0.0;
  for (size_t i = 0; i < Q.size(); ++i) {
    if (max_q < Q.at(i).second || i == 0) {
      max_q = Q.at(i).second;
      theta_star = Q.at(i).first;
    }
  }
  const float32_t sin_theta_star = std::sin(theta_star);
  const float32_t cos_theta_star = std::cos(theta_star);

  PointT e_1_star;  // col.11, Algo.2
  xr_(e_1_star) = cos_theta_star;
  yr_(e_1_star) = sin_theta_star;
  PointT e_2_star;
  xr_(e_2_star) = -sin_theta_star;
  yr_(e_2_star) = cos_theta_star;

  std::vector<float32_t> C_1_star;  // col.11, Algo.2
  std::vector<float32_t> C_2_star;  // col.11, Algo.2

  for (auto it = begin; it != end; ++it) {
    const PointT & pt = *it;
    C_1_star.push_back(x_(pt) * x_(e_1_star) + y_(pt) * y_(e_1_star));
    C_2_star.push_back(x_(pt) * x_(e_2_star) + y_(pt) * y_(e_1_star));
  }

  // col.12, Algo.2
  const float32_t min_C_1_star = *std::min_element(C_1_star.begin(), C_1_star.end());
  const float32_t max_C_1_star = *std::max_element(C_1_star.begin(), C_1_star.end());
  const float32_t min_C_2_star = *std::min_element(C_2_star.begin(), C_2_star.end());
  const float32_t max_C_2_star = *std::max_element(C_2_star.begin(), C_2_star.end());

  const float32_t a_1 = cos_theta_star;
  const float32_t b_1 = sin_theta_star;
  const float32_t c_1 = min_C_1_star;
  const float32_t a_2 = -1.0F * sin_theta_star;
  const float32_t b_2 = cos_theta_star;
  const float32_t c_2 = min_C_2_star;
  const float32_t a_3 = cos_theta_star;
  const float32_t b_3 = sin_theta_star;
  const float32_t c_3 = max_C_1_star;
  const float32_t a_4 = -1.0F * sin_theta_star;
  const float32_t b_4 = cos_theta_star;
  const float32_t c_4 = max_C_2_star;

  // calc center of bounding box
  float32_t intersection_x_1 = (b_1 * c_2 - b_2 * c_1) / (a_2 * b_1 - a_1 * b_2);
  float32_t intersection_y_1 = (a_1 * c_2 - a_2 * c_1) / (a_1 * b_2 - a_2 * b_1);
  float32_t intersection_x_2 = (b_3 * c_4 - b_4 * c_3) / (a_4 * b_3 - a_3 * b_4);
  float32_t intersection_y_2 = (a_3 * c_4 - a_4 * c_3) / (a_3 * b_4 - a_4 * b_3);

  // calc dimension of bounding box
  PointT e_x;
  xr_(e_x) = a_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1));
  yr_(e_x) = b_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1));
  PointT e_y;
  xr_(e_y) = a_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2));
  yr_(e_y) = b_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2));

  PointT diagonal_vec;
  xr_(diagonal_vec) = intersection_x_1 - intersection_x_2;
  yr_(diagonal_vec) = intersection_y_1 - intersection_y_2;
  // calc yaw
  tf2::Quaternion quat;
  quat.setEuler(/* roll */ 0, /* pitch */ 0, /* yaw */ std::atan2(y_(e_1_star), x_(e_1_star)));

  BoundingBox bbox;
  bbox.size.x = std::fabs(dot_2d(e_x, diagonal_vec));
  bbox.size.y = std::fabs(dot_2d(e_y, diagonal_vec));
  bbox.size.z = std::max((max_z - min_z), epsilon);
  bbox.centroid.x = (intersection_x_1 + intersection_x_2) * 0.5F;
  bbox.centroid.y = (intersection_y_1 + intersection_y_2) * 0.5F;
  bbox.centroid.z = min_z + bbox.size.z * 0.5F;
  bbox.orientation.x = static_cast<float32_t>(quat.getX());
  bbox.orientation.y = static_cast<float32_t>(quat.getY());
  bbox.orientation.z = static_cast<float32_t>(quat.getZ());
  bbox.orientation.w = static_cast<float32_t>(quat.getW());

  // check wrong output
  bbox.size.x = std::max(bbox.size.x, epsilon);
  bbox.size.y = std::max(bbox.size.y, epsilon);

  return bbox;
}
}  // namespace details


template <typename IT>
BoundingBox l_shape_fit_bounding_box_2d(const IT begin, const IT end)
{
  float32_t min_angle = 0.0F;
  float32_t max_angle = static_cast<float32_t>(M_PI / 2);

  return details::fit_l_shape_fitting(
    begin, end, min_angle, max_angle);
}
}  // namespace bounding_box
}  // namespace geometry
}  // namespace common
}  // namespace autoware

#endif  // BUILD_SRC_COMMON_AUTOWARE_AUTO_GEOMETRY_INCLUDE_GEOMETRY_BOUNDING_BOX_L_SHAPE_FITTING_H_
