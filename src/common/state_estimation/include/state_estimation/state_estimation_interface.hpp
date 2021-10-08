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


#ifndef STATE_ESTIMATION__STATE_ESTIMATION_INTERFACE_HPP_
#define STATE_ESTIMATION__STATE_ESTIMATION_INTERFACE_HPP_


#include <helper_functions/crtp.hpp>
#include <state_estimation/measurement/measurement_interface.hpp>
#include <state_estimation/visibility_control.hpp>
#include <state_vector/generic_state.hpp>

#include <chrono>

namespace autoware
{
namespace common
{
namespace state_estimation
{

///
/// @brief      Interface for a filter that can be used to track an object.
///
/// @tparam     Derived  An implementation of this filter.
///
template<typename Derived>
class STATE_ESTIMATION_PUBLIC StateEstimationInterface
  : public common::helper_functions::crtp<Derived>
{
public:
  ///
  /// @brief      Predict the state by dt into the future.
  ///
  /// @param[in]  state   The state before prediction.
  /// @param[in]  dt      Time for prediction.
  ///
  /// @tparam     StateT  Type of the state.
  ///
  /// @return     A new predicted state.
  ///
  template<typename StateT>
  state_vector::CovarianceAnd<StateT> predict(
    const state_vector::CovarianceAnd<StateT> & state,
    const std::chrono::nanoseconds & dt) const
  {
    return this->impl().crtp_predict(state, dt);
  }

  ///
  /// @brief      Correct the state with a measurement.
  ///
  /// @note       It is expected that prediction step was called before-wise.
  ///
  /// @param[in]  state         The state before correction
  /// @param[in]  measurement   The measurement
  ///
  /// @tparam     StateT        The type of the state.
  /// @tparam     MeasurementT  Measurement type
  ///
  /// @return     A corrected state.
  ///
  template<typename StateT, typename MeasurementT>
  state_vector::CovarianceAnd<StateT> correct(
    const state_vector::CovarianceAnd<StateT> state,
    const MeasurementT & measurement) const
  {
    static_assert(
      std::is_base_of<MeasurementInterface<MeasurementT>, MeasurementT>::value,
      "\n\nMeasurement must inherit from MeasurementInterface\n\n");
    return this->impl().template crtp_correct<StateT, MeasurementT>(state, measurement);
  }
};


}  // namespace state_estimation
}  // namespace common
}  // namespace autoware

#endif  // STATE_ESTIMATION__STATE_ESTIMATION_INTERFACE_HPP_
