// Copyright 2021 The Autoware Foundation
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
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

// Software License Agreement (BSD License)
//
// Copyright (c) 2016, Guan-Horng Liu.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author:  Guan-Horng Liu

// Software License Agreement (BSD License)
//
//  Copyright (c) 2010, Rice University
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   * Neither the name of the Rice University nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
#ifndef FREESPACE_PLANNER__REEDS_SHEPP_HPP_
#define FREESPACE_PLANNER__REEDS_SHEPP_HPP_

#include <freespace_planner/reeds_shepp_impl.hpp>
#include <freespace_planner/visibility_control.hpp>


namespace autoware
{
namespace planning
{
namespace freespace_planner
{
/// \brief Input position and orientation representation
struct FREESPACE_PLANNER_PUBLIC StateXYT
{
  double x;
  double y;
  double yaw;
};

/// \class ReedsShepp
/// \brief Class calculating shortest path on plane, considering arc and straight, forward and
/// backward movements.
///        The algorithm analyzes a sufficient subset of situations where robot can move clockwise,
///        counterclockwise, straight in forward or backward direction. The clockwise and
///        counterclockwise turns can be generalized by substituting the left/right nomenclature
///        with an arc movement marked by the 'C' character. A straight movement can be marked as
///        the 'S'. The situation "(...)CC(...)" means a change in the direction of the turn,
///        regardless of whether the first turn is counterclockwise or clockwise. This class uses
///        implementation of algorithm described by J. A. REEDS and L. A. SHEPP. For more detailed
///        info please refer to "Optimal paths for a car that goes both forwards and backwards"
///        paper.
class FREESPACE_PLANNER_PUBLIC ReedsShepp
{
public:
  explicit ReedsShepp(double turningRadius)
  : turning_radius_(turningRadius) {}

  /// \brief Return the shortest distance between state 0 and state 1
  double distance(const StateXYT & s0, const StateXYT & s1);

  /// \brief Return the shortest Reeds-Shepp path from state 0 to state 1
  ReedsSheppPath reedsShepp(const StateXYT & s0, const StateXYT & s1);

protected:
  ReedsSheppPath reedsShepp(ReedsSheppNode node);

  /** \brief Turning radius */
  double turning_radius_;
};

}  // namespace freespace_planner
}  // namespace planning
}  // namespace autoware

#endif  // FREESPACE_PLANNER__REEDS_SHEPP_HPP_
