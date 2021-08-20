// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.


#include <tracking/box_shape_tracker.hpp>

#include <autoware_auto_msgs/msg/shape.hpp>
#include <motion_model/stationary_motion_model.hpp>
#include <state_estimation/kalman_filter/kalman_filter.hpp>
#include <state_estimation/measurement/linear_measurement.hpp>
#include <state_estimation/noise_model/uniform_noise.hpp>
#include <state_vector/common_variables.hpp>
#include <state_vector/variable.hpp>

#include <limits>

namespace autoware
{
namespace perception
{
namespace tracking
{

constexpr common::types::float32_t BoxShapeTracker::kInitialStandardDeviation;


void BoxShapeTracker::update(
  const autoware_auto_msgs::msg::Shape & shape,
  const float standard_deviation)
{
  if (shape.polygon.points.size() != 4U) {
    throw std::domain_error("Shape must have exactly 4 points to use BoxShapeTracker class.");
  }
  const auto measurement_height = Measurement<StateHeight>::create_with_stddev(
    StateHeight::Vector{shape.height}, StateHeight::Vector{standard_deviation});
  m_height_filter.correct(measurement_height);
  for (auto i = 0UL; i < m_xyz_filters.size(); ++i) {
    const auto & point = shape.polygon.points[i];
    const auto measurement_xyz = Measurement<StateXyz>::create_with_stddev(
      {point.x, point.y, point.z},
      {standard_deviation, standard_deviation, standard_deviation});
    m_xyz_filters[i].correct(measurement_xyz);
  }
}

autoware_auto_msgs::msg::Shape BoxShapeTracker::current_shape() const noexcept
{
  autoware_auto_msgs::msg::Shape shape;
  for (const auto & filter : m_xyz_filters) {
    geometry_msgs::msg::Point32 point;
    point.x = filter.state().at<X>();
    point.y = filter.state().at<Y>();
    point.z = filter.state().at<Z>();
    shape.polygon.points.push_back(point);
  }
  shape.height = m_height_filter.state().at<HEIGHT>();
  return shape;
}


}  // namespace tracking
}  // namespace perception
}  // namespace autoware
