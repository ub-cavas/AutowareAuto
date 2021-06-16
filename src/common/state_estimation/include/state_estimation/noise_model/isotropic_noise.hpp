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

#ifndef STATE_ESTIMATION__NOISE_MODEL__ISOTROPIC_NOISE_HPP_
#define STATE_ESTIMATION__NOISE_MODEL__ISOTROPIC_NOISE_HPP_

#include <state_estimation/noise_model/noise_interface.hpp>
#include <state_estimation/visibility_control.hpp>
#include <state_vector/common_states.hpp>

#include <algorithm>
#include <array>
#include <vector>

namespace autoware
{
namespace common
{
namespace state_estimation
{

///
/// @brief      A class that describes the Wiener process noise.
///
///             For more details see notebook here:
///             https://nbviewer.jupyter.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/
///             blob/master/07-Kalman-Filter-Math.ipynb#Piecewise-White-Noise-Model (combine into
///             one line)
///
/// @tparam     StateT  A given state type.
///
template<typename StateT>
class STATE_ESTIMATION_PUBLIC IsotropicNoise : public NoiseInterface<IsotropicNoise<StateT>>
{
public:
  using State = StateT;

  ///
  /// @brief      Constructor from acceleration variances.
  ///
  /// @param[in]  acceleration_variances  The acceleration variances, note that these are sigmas,
  ///                                     not sigmas squared. Note that while this array has place
  ///                                     for all the variables, it should only hold those
  ///                                     representing acceleration values. The positions of these
  ///                                     variables in the array do not represent their position in
  ///                                     the actual state vector and should start from the start of
  ///                                     this array.
  ///
  explicit IsotropicNoise(const typename StateT::Scalar & variance)
  : m_variance{variance} {}

protected:
  // Required to allow the crtp interface call the following functions.
  friend NoiseInterface<IsotropicNoise<StateT>>;

  ///
  /// @brief      A CRTP-called covariance getter.
  ///
  /// @return     A covariance of the noise process over given time.
  ///
  typename State::Matrix crtp_covariance(const std::chrono::nanoseconds &) const
  {
    return m_variance * State::Matrix::Identity();
  }

private:
  typename StateT::Scalar m_variance{};
};

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware

#endif  // STATE_ESTIMATION__NOISE_MODEL__ISOTROPIC_NOISE_HPP_
