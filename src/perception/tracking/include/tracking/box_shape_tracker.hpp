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


#ifndef TRACKING__BOX_SHAPE_TRACKER_HPP_
#define TRACKING__BOX_SHAPE_TRACKER_HPP_

#include <autoware_auto_msgs/msg/shape.hpp>
#include <motion_model/stationary_motion_model.hpp>
#include <state_estimation/kalman_filter/kalman_filter.hpp>
#include <state_estimation/measurement/linear_measurement.hpp>
#include <state_estimation/noise_model/uniform_noise.hpp>
#include <state_vector/common_variables.hpp>
#include <state_vector/variable.hpp>
#include <tracking/visibility_control.hpp>

#include <limits>

namespace autoware
{
namespace perception
{
namespace tracking
{

class TRACKING_PUBLIC BoxShapeTracker
{
  using X = common::state_vector::variable::X;
  using Y = common::state_vector::variable::Y;
  using Z = common::state_vector::variable::Z;

  struct HEIGHT : public common::state_vector::Variable {};

  using StateXyz = common::state_vector::FloatState<X, Y, Z>;
  using StateHeight = common::state_vector::FloatState<HEIGHT>;

  template<typename StateT>
  using MotionModel = common::motion_model::StationaryMotionModel<StateT>;
  template<typename StateT>
  using NoiseModel = common::state_estimation::UniformNoise<StateT>;

  template<typename StateT>
  using KalmanFilter =
    common::state_estimation::KalmanFilter<MotionModel<StateT>, NoiseModel<StateT>>;
  template<typename StateT>
  using Measurement = common::state_estimation::LinearMeasurement<StateT>;

  static constexpr common::types::float32_t kInitialStandardDeviation = 100000000.0F;

public:
  void update(const autoware_auto_msgs::msg::Shape & shape, const float standard_deviation);

  autoware_auto_msgs::msg::Shape current_shape() const noexcept;

private:
  MotionModel<StateXyz> m_xyz_motion_model;
  MotionModel<StateHeight> m_height_motion_model;

  NoiseModel<StateXyz> m_xyz_noise_model{StateXyz::Matrix::Identity()};
  NoiseModel<StateHeight> m_height_noise_model{StateHeight::Matrix::Identity()};

  KalmanFilter<StateHeight> m_height_filter {
    common::state_estimation::make_kalman_filter(
      m_height_motion_model, m_height_noise_model, StateHeight{},
      kInitialStandardDeviation * StateHeight::Matrix::Identity())};
  std::array<KalmanFilter<StateXyz>, 4UL> m_xyz_filters {
    common::state_estimation::make_kalman_filter(
      m_xyz_motion_model, m_xyz_noise_model, StateXyz{},
      kInitialStandardDeviation * StateXyz::Matrix::Identity()),
    common::state_estimation::make_kalman_filter(
      m_xyz_motion_model, m_xyz_noise_model, StateXyz{},
      kInitialStandardDeviation * StateXyz::Matrix::Identity()),
    common::state_estimation::make_kalman_filter(
      m_xyz_motion_model, m_xyz_noise_model, StateXyz{},
      kInitialStandardDeviation * StateXyz::Matrix::Identity()),
    common::state_estimation::make_kalman_filter(
      m_xyz_motion_model, m_xyz_noise_model, StateXyz{},
      kInitialStandardDeviation * StateXyz::Matrix::Identity()),
  };
};


}  // namespace tracking
}  // namespace perception
}  // namespace autoware

#endif  // TRACKING__BOX_SHAPE_TRACKER_HPP_
