/*
 * Copyright 2021 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the class for quintic Bézier curves.

#ifndef INTERACTIVE_TRAJECTORY_SPOOFER__BEZIER_HPP_
#define INTERACTIVE_TRAJECTORY_SPOOFER__BEZIER_HPP_

#include <interactive_trajectory_spoofer/visibility_control.hpp>

#include "common/types.hpp"

#include <array>
#include <vector>
#include <eigen3/Eigen/Core>
#include <iostream>

namespace autoware
{
namespace interactive_trajectory_spoofer
{
using autoware::common::types::float64_t;
typedef Eigen::Matrix<float64_t, 2, 1> Point;
typedef Eigen::Matrix<float64_t, 3, 1> PointWithHeading;

// Coefficients for matrix calculation of the quintic Bézier curve.
static const Eigen::Matrix<float64_t, 6, 6> quintic_bezier_coefficients((Eigen::Matrix<float64_t, 6,
  6>() <<
  1, 0, 0, 0, 0, 0,
  -5, 5, 0, 0, 0, 0,
  10, -20, 10, 0, 0, 0,
  -10, 30, -30, 10, 0, 0,
  5, -20, 30, -20, 5, 0,
  -1, 5, -10, 10, -5, 1).finished());
const Eigen::Matrix<float64_t, 5, 6> quintic_bezier_velocity_coefficients((Eigen::Matrix<float64_t,
  5, 6>() <<
  quintic_bezier_coefficients.row(1) * 1,
  quintic_bezier_coefficients.row(2) * 2,
  quintic_bezier_coefficients.row(3) * 3,
  quintic_bezier_coefficients.row(4) * 4,
  quintic_bezier_coefficients.row(5) * 5).finished());
const Eigen::Matrix<float64_t, 4, 6> quintic_bezier_acceleration_coefficients(
  (Eigen::Matrix<float64_t, 4, 6>() <<
    quintic_bezier_velocity_coefficients.row(1) * 1,
    quintic_bezier_velocity_coefficients.row(2) * 2,
    quintic_bezier_velocity_coefficients.row(3) * 3,
    quintic_bezier_velocity_coefficients.row(4) * 4).finished());

/// \class Quintic Bezier curve
/// \brief Class representing quintic Bezier curves defined by 6 control points
class INTERACTIVE_TRAJECTORY_SPOOFER_PUBLIC Bezier
{
  Eigen::Matrix<float64_t, 6, 2> m_control_points;

public:
  //@brief empty constructor
  Bezier() = default;
  //@brief constructor from a matrix
  Bezier(const Eigen::Matrix<float64_t, 6, 2> & control_points);
  //@brief constructor from a set of control points
  Bezier(const std::vector<Point> & control_points);
  //@brief return the control points
  Eigen::Matrix<float64_t, 6, 2> getControlPoints() const;
  //@brief update the ith control points
  void updateControlPoint(int64_t i, Point new_point);
  //@brief return the curve in cartersian frame with the desired resolution
  std::vector<Point> cartesian(const float64_t resolution) const;
  //@brief return the curve in cartersian frame with the desired number of points
  std::vector<Point> cartesian(const size_t nb_points) const;
  //@brief return the curve in cartersian frame (including angle) with the desired number of points
  std::vector<PointWithHeading> cartesianWithHeading(const size_t nb_points) const;
  //@brief calculate the curve value for the given parameter t
  Point value(const float64_t t) const;
  //@brief calculate the curve value for the given parameter t (using matrix formulation)
  Point valueM(const float64_t t) const;
  //@brief calculate the velocity (1st derivative) for the given parameter t
  Point velocity(const float64_t t) const;
  //@brief calculate the acceleration (2nd derivative) for the given parameter t
  Point acceleration(const float64_t t) const;
  //@breif return the heading (in radians) of the tangent for the given parameter t
  float64_t heading(const float64_t t) const;
  //@brief calculate the curvature for the given parameter t
  float64_t curvature(const float64_t t) const;
};
}  // namespace interactive_trajectory_spoofer
}  // namespace autoware

#endif  // INTERACTIVE_TRAJECTORY_SPOOFER__BEZIER_HPP_
