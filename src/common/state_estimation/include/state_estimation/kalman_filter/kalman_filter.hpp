// Copyright 2021 Apex.AI, Inc.
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

#ifndef STATE_ESTIMATION__KALMAN_FILTER__KALMAN_FILTER_HPP_
#define STATE_ESTIMATION__KALMAN_FILTER__KALMAN_FILTER_HPP_

#include <helper_functions/float_comparisons.hpp>
#include <motion_model/motion_model_interface.hpp>
#include <motion_model/stationary_motion_model.hpp>
#include <state_estimation/noise_model/noise_interface.hpp>
#include <state_estimation/state_estimation_interface.hpp>
#include <state_estimation/visibility_control.hpp>

#include <Eigen/LU>

#include <limits>
#include <vector>

namespace autoware
{
namespace common
{
namespace state_estimation
{
///
/// @brief      A Kalman filter implementation.
///
/// @tparam     MotionModelT  Type of the motion model.
/// @tparam     NoiseModelT   Type of the noise model.
///
template<typename MotionModelT, typename NoiseModelT>
class STATE_ESTIMATION_PUBLIC KalmanFilter
  : public StateEstimationInterface<KalmanFilter<MotionModelT, NoiseModelT>>
{
  static_assert(
    std::is_base_of<common::motion_model::MotionModelInterface<MotionModelT>, MotionModelT>::value,
    "\n\nMotion model must inherit from MotionModelInterface\n\n");
  static_assert(
    std::is_base_of<NoiseInterface<NoiseModelT>, NoiseModelT>::value,
    "\n\nNoise model must inherit from NoiseInterface\n\n");
  static_assert(
    std::is_same<typename MotionModelT::State, typename NoiseModelT::State>::value,
    "\n\nMotion model and noise model must have the same underlying state\n\n");

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using State = typename MotionModelT::State;
  using StateMatrix = typename State::Matrix;
  using MotionModel = MotionModelT;
  using NoiseModel = NoiseModelT;

  ///
  /// @brief      Constructs a new instance of a Kalman filter.
  ///
  /// @param[in]  motion_model        The motion model to be used to predict the movement.
  /// @param[in]  noise_model         The noise model that models the motion noise.
  ///
  explicit KalmanFilter(MotionModelT motion_model, NoiseModelT noise_model)
  : m_motion_model{motion_model}, m_noise_model{noise_model} {}

  ///
  /// @brief      Predict next state.
  ///
  /// @param[in]  state   The state before prediction.
  /// @param[in]  dt      Time difference to the time at which prediction is needed.
  ///
  /// @tparam     StateT  Type of the state.
  ///
  /// @return     Predicted state.
  ///
  template<typename StateT>
  state_vector::CovarianceAnd<StateT> crtp_predict(
    const state_vector::CovarianceAnd<StateT> & state, const std::chrono::nanoseconds & dt) const
  {
    const auto predicted_state = m_motion_model.predict(state.state, dt);
    const auto & motion_jacobian = m_motion_model.jacobian(predicted_state, dt);
    const auto predicted_covariance =
      motion_jacobian * state.covariance * motion_jacobian.transpose() +
      m_noise_model.covariance(dt);
    return {predicted_state, predicted_covariance};
  }

  ///
  /// @brief      Correct the predicted state given a measurement
  ///
  /// @note       It is expected that a prediction step was done right before the correction.
  ///
  /// @param[in]  state         The state before correction
  /// @param[in]  measurement   Current measurement.
  ///
  /// @tparam     StateT        Type of the state.
  /// @tparam     MeasurementT  Measurement type.
  ///
  /// @return     State corrected with the measurement.
  ///
  template<typename StateT, typename MeasurementT>
  state_vector::CovarianceAnd<StateT> crtp_correct(
    const state_vector::CovarianceAnd<StateT> & state, const MeasurementT & measurement) const
  {
    const auto expected_measurement = measurement.create_new_instance_from(state.state);
    const auto innovation = wrap_all_angles(measurement.state() - expected_measurement);
    const auto mapping_matrix = measurement.mapping_matrix_from(state.state);
    const auto innovation_covariance =
      mapping_matrix * state.covariance * mapping_matrix.transpose() + measurement.covariance();
    const auto kalman_gain =
      state.covariance * mapping_matrix.transpose() * innovation_covariance.inverse();
    auto corrected_state = state.state + kalman_gain * innovation.vector();
    corrected_state.wrap_all_angles();
    const auto corrected_covariance =
      (State::Matrix::Identity() - kalman_gain * mapping_matrix) * state.covariance;
    return {corrected_state, corrected_covariance};
  }

private:
  /// Motion model used to predict the state forward.
  MotionModelT m_motion_model{};
  /// Noise model of the movement.
  NoiseModelT m_noise_model{};
};

///
/// @brief      A utility function that creates a Kalman filter.
///
/// @details    Mostly this is needed to avoid passing the template parameters explicitly and let
///             the compiler infer them from the objects passed into this function.
///
/// @param[in]  motion_model        A motion model.
/// @param[in]  noise_model         A noise model.
///
/// @tparam     MotionModelT        Type of the motion model.
/// @tparam     NoiseModelT         Type of the noise model.
///
/// @return     Returns a valid KalmanFilter instance.
///
template<typename MotionModelT, typename NoiseModelT>
auto make_kalman_filter(
  const MotionModelT & motion_model,
  const NoiseModelT & noise_model)
{
  return KalmanFilter<MotionModelT, NoiseModelT>{motion_model, noise_model};
}

///
/// @brief      A utility function that creates a Kalman filter that is to be used for correction
///             only, i.e., this Kalman filter cannot predict the state forward in time.
///
/// @details    Mostly this is needed to avoid passing the template parameters explicitly and let
///             the compiler infer them from the objects passed into this function.
///
/// @param[in]  initial_state       The initial state
/// @param[in]  initial_covariance  The initial covariance
///
/// @tparam     StateT              { description }
/// @tparam     MotionModelT  Type of the motion model.
/// @tparam     NoiseModelT   Type of the noise model.
///
/// @return     Returns a valid KalmanFilter instance.
///
template<typename StateT>
auto make_correction_only_kalman_filter()
{
  struct DummyNoise : public NoiseInterface<DummyNoise>
  {
    using State = StateT;
    typename State::Matrix crtp_covariance(const std::chrono::nanoseconds &) const
    {
      throw std::runtime_error(
              "Trying to use a correction-only Kalman filter to predict the state.");
    }
  };

  using MotionModel = common::motion_model::StationaryMotionModel<StateT>;

  return make_kalman_filter(MotionModel{}, DummyNoise{});
}

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware

#endif  // STATE_ESTIMATION__KALMAN_FILTER__KALMAN_FILTER_HPP_
