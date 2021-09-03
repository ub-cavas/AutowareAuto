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

#include <interactive_trajectory_spoofer/bezier.hpp>

namespace autoware
{
namespace interactive_trajectory_spoofer
{
Bezier::Bezier(const Eigen::Matrix<float64_t, 6, 2> & control_points)
: m_control_points(control_points)
{
}
Bezier::Bezier(const std::vector<Point> & control_points)
{
  if (control_points.size() != 6) {
    //TODO exception
    std::cerr << "Trying to initialize a quintic bezier curve with " << control_points.size() <<
      " control points." << std::endl;
  }
  m_control_points << control_points[0], control_points[1], control_points[2],
    control_points[3], control_points[4], control_points[5];
}

Eigen::Matrix<float64_t, 6, 2> Bezier::getControlPoints() const
{
  return m_control_points;
}

void Bezier::updateControlPoint(int64_t i, Point new_point)
{
  m_control_points.row(i) = new_point;
}

Point Bezier::value(const float64_t t) const
{
  Point point = {0.0, 0.0};
  // sum( binomial(i in 5) * (1 - t)^(5-i) * t^i * control_point[i] )
  point += std::pow((1 - t), 5) * m_control_points.row(0);
  point += 5 * std::pow((1 - t), 4) * t * m_control_points.row(1);
  point += 10 * std::pow((1 - t), 3) * t * t * m_control_points.row(2);
  point += 10 * std::pow((1 - t), 2) * t * t * t * m_control_points.row(3);
  point += 5 * (1 - t) * t * t * t * t * m_control_points.row(4);
  point += t * t * t * t * t * m_control_points.row(5);
  return point;
}

Point Bezier::valueM(const float64_t t) const
{
  Eigen::Matrix<float64_t, 1, 6> ts;
  ts << 1, t, t * t, t * t * t, t * t * t * t, t * t * t * t * t;
  return ts * quintic_bezier_coefficients * m_control_points;
}

// TODO ensure points are separated by fixed arc-length (rather than fixed t-parameter)
std::vector<Point> Bezier::cartesian(const size_t nb_points) const
{
  std::vector<Point> points;
  points.reserve(nb_points);
  const float64_t step = 1.0 / static_cast<float64_t>(nb_points - 1);
  for (float64_t t = 0.0; t <= 1.0; t += step) {
    points.push_back(valueM(t));
  }
  return points;
}

// TODO ensure points are separated by fixed arc-length (rather than fixed t-parameter)
std::vector<Point> Bezier::cartesian(const float64_t resolution) const
{
  std::vector<Point> points;
  points.reserve(static_cast<size_t>(1 / resolution));
  for (float64_t t = 0.0; t <= 1.0; t += resolution) {
    points.push_back(valueM(t));
  }
  return points;
}

// TODO ensure points are separated by fixed arc-length (rather than fixed t-parameter)
std::vector<PointWithHeading> Bezier::cartesianWithHeading(const size_t nb_points) const
{
  std::vector<PointWithHeading> points;
  points.reserve(nb_points);
  const float64_t step = 1.0 / static_cast<float64_t>(nb_points - 1);
  for (float64_t t = 0.0; t <= 1.0; t += step) {
    const Point point = valueM(t);
    points.emplace_back(point.x(), point.y(), heading(t));
  }
  return points;
}

Point Bezier::velocity(const float64_t t) const
{
  Eigen::Matrix<float64_t, 1, 5> ts;
  ts << 1, t, t * t, t * t * t, t * t * t * t;
  return ts * quintic_bezier_velocity_coefficients * m_control_points;
}

Point Bezier::acceleration(const float64_t t) const
{
  Eigen::Matrix<float64_t, 1, 4> ts;
  ts << 1, t, t * t, t * t * t;
  return ts * quintic_bezier_acceleration_coefficients * m_control_points;
}

float64_t Bezier::curvature(const float64_t t) const
{
  const Point vel = velocity(t);
  const Point accel = acceleration(t);
  const float64_t denominator = std::pow(vel.x() * vel.x() + vel.y() * vel.y(), 3.0 / 2.0);
  if (denominator) {
    return (vel.x() * accel.y() - accel.x() * vel.y()) / denominator;
  } else {
    return std::numeric_limits<float64_t>::infinity();
  }
}

float64_t Bezier::heading(float64_t t) const
{
  const Point vel = velocity(t);
  return std::atan2(vel.y(), vel.x());
}
}  // namespace interactive_trajectory_spoofer
}  // namespace autoware
