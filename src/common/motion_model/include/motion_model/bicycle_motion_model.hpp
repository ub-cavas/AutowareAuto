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
// Developed by Apex.AI, Inc.

#ifndef MOTION_MODEL__BICYCLE_MOTION_MODEL_HPP_
#define MOTION_MODEL__BICYCLE_MOTION_MODEL_HPP_

#include <motion_model/motion_model_interface.hpp>
#include <motion_model/visibility_control.hpp>
#include <state_vector/common_states.hpp>
#include <state_vector/generic_state.hpp>
#include <state_vector/common_variables.hpp>

namespace autoware
{
namespace common
{
namespace motion_model
{

template<typename ScalarT>
struct SteeringAngle
{
  ScalarT steering_angle;
};

///
/// @brief      A bicycle motion model class.
///
template<typename ScalarT>
class MOTION_MODEL_PUBLIC BicycleMotionModel
  : public MotionModelInterface<BicycleMotionModel<ScalarT>>
{
public:
  using State = state_vector::GenericState<ScalarT, state_vector::variable::XY_VELOCITY,
      state_vector::variable::XY_ACCELERATION, state_vector::variable::SIDE_SLIP_ANGLE,
      state_vector::variable::YAW_CHANGE_RATE>;
  using Control = SteeringAngle<ScalarT>;

protected:
  // Allow the CRTP interface to call private functions.
  friend MotionModelInterface<BicycleMotionModel<ScalarT>>;

  ///
  /// @brief      A crtp-called function that predicts the state forward.
  ///
  /// @param[in]  state  The current state vector
  /// @param[in]  dt     Time difference
  ///
  /// @return     New state after prediction.
  ///
  inline State crtp_predict(
    const State & state,
    const Control & control,
    const std::chrono::nanoseconds & dt) const
  {
    State result {state.vector()};
    // nonsense
    (void) dt;
    result.template at<state_vector::variable::YAW_CHANGE_RATE>() = control.steering_angle;
    return result;
  }

  ///
  /// @brief      A crtp-called function that computes a Jacobian.
  ///
  /// @note       The default implementation assumes that all variables have position, velocity and
  ///             acceleration entries. If a custom state that does not follow this convention is to
  ///             be used a specialization of this function must be added.
  ///
  /// @return     A matrix that represents the Jacobian.
  ///
  typename State::Matrix crtp_jacobian(
    const State &, const Control &,
    const std::chrono::nanoseconds &) const
  {
    return State::Matrix::Identity();
  }

private:
  ScalarT l_r{};
  ScalarT l_f{};
};

}  // namespace motion_model
}  // namespace common
}  // namespace autoware

#endif  // MOTION_MODEL__BICYCLE_MOTION_MODEL_HPP_
