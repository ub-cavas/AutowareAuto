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

#include "freespace_planner/reeds_shepp_impl.hpp"

#include <cassert>
#include <cmath>
#include <limits>


// The comments, variable names, etc. use the nomenclature from the Reeds & Shepp paper.
const double PI = M_PI;
const double EPSILON = 1e-6;
const double NUMERIC_ZERO = 10.0 * std::numeric_limits<double>::epsilon();

void toPolarCoordinates(double x, double y, double & radius, double & theta)
{
  radius = std::hypot(x, y);
  theta = std::atan2(y, x);
}

void calculateTauAndOmega(
  double u, double v, double xi, double eta, double phi, double & tau, double & omega)
{
  double delta = autoware::common::helper_functions::wrap_angle(u - v);

  double A = std::sin(u) - std::sin(delta);
  double B = std::cos(u) - std::cos(delta) - 1.0;

  double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
  double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;

  tau = (t2 < 0.0) ?
    autoware::common::helper_functions::wrap_angle(t1 + PI) :
    autoware::common::helper_functions::wrap_angle(t1);
  omega = autoware::common::helper_functions::wrap_angle(tau - u + v - phi);
}

double lengthFromParameters(double param_t, double param_u, double param_v)
{
  return std::abs(param_t) + std::abs(param_u) + std::abs(param_v);
}

namespace autoware
{
namespace planning
{
namespace freespace_planner
{
const ReedsSheppPathSegmentType RP_PATH_TYPE[18][5] = {
  {COUNTERCLOCKWISE, CLOCKWISE, COUNTERCLOCKWISE, NO_OPERATION, NO_OPERATION},  // 0
  {CLOCKWISE, COUNTERCLOCKWISE, CLOCKWISE, NO_OPERATION, NO_OPERATION},         // 1
  {COUNTERCLOCKWISE, CLOCKWISE, COUNTERCLOCKWISE, CLOCKWISE, NO_OPERATION},     // 2
  {CLOCKWISE, COUNTERCLOCKWISE, CLOCKWISE, COUNTERCLOCKWISE, NO_OPERATION},     // 3
  {COUNTERCLOCKWISE, CLOCKWISE, STRAIGHT, COUNTERCLOCKWISE, NO_OPERATION},      // 4
  {CLOCKWISE, COUNTERCLOCKWISE, STRAIGHT, CLOCKWISE, NO_OPERATION},             // 5
  {COUNTERCLOCKWISE, STRAIGHT, CLOCKWISE, COUNTERCLOCKWISE, NO_OPERATION},      // 6
  {CLOCKWISE, STRAIGHT, COUNTERCLOCKWISE, CLOCKWISE, NO_OPERATION},             // 7
  {COUNTERCLOCKWISE, CLOCKWISE, STRAIGHT, CLOCKWISE, NO_OPERATION},             // 8
  {CLOCKWISE, COUNTERCLOCKWISE, STRAIGHT, COUNTERCLOCKWISE, NO_OPERATION},      // 9
  {CLOCKWISE, STRAIGHT, CLOCKWISE, COUNTERCLOCKWISE, NO_OPERATION},             // 10
  {COUNTERCLOCKWISE, STRAIGHT, COUNTERCLOCKWISE, CLOCKWISE, NO_OPERATION},      // 11
  {COUNTERCLOCKWISE, STRAIGHT, CLOCKWISE, NO_OPERATION, NO_OPERATION},          // 12
  {CLOCKWISE, STRAIGHT, COUNTERCLOCKWISE, NO_OPERATION, NO_OPERATION},          // 13
  {COUNTERCLOCKWISE, STRAIGHT, COUNTERCLOCKWISE, NO_OPERATION, NO_OPERATION},   // 14
  {CLOCKWISE, STRAIGHT, CLOCKWISE, NO_OPERATION, NO_OPERATION},                 // 15
  {COUNTERCLOCKWISE, CLOCKWISE, STRAIGHT, COUNTERCLOCKWISE, CLOCKWISE},         // 16
  {CLOCKWISE, COUNTERCLOCKWISE, STRAIGHT, CLOCKWISE, COUNTERCLOCKWISE}          // 17
};

ReedsSheppPath::ReedsSheppPath(
  const ReedsSheppPathSegmentType * type, double t, double u, double v, double w, double x)
: type_(type),
  length_({t, u, v, w, x}),
  totalLength_(std::abs(t) + std::abs(u) + std::abs(v) + std::abs(w) + std::abs(x))
{
}

namespace reeds_shepp
{
namespace
{
// formula 8.1 in Reeds-Shepp paper
/// \brief Function used in CSC variant
bool LpSpLp(ReedsSheppNode node, double & t, double & u, double & v)
{
  toPolarCoordinates(node.x - std::sin(node.phi), node.y - 1.0 + std::cos(node.phi), u, t);
  if (t >= -NUMERIC_ZERO) {
    v = autoware::common::helper_functions::wrap_angle(node.phi - t);
    if (v >= -NUMERIC_ZERO) {
      assert(std::abs(u * std::cos(t) + std::sin(node.phi) - node.x) < EPSILON);
      assert(std::abs(u * std::sin(t) - std::cos(node.phi) + 1.0 - node.y) < EPSILON);
      assert(
        std::abs(autoware::common::helper_functions::wrap_angle(t + v - node.phi)) < EPSILON);
      return true;
    }
  }
  return false;
}

// formula 8.2 in Reeds-Shepp paper
/// \brief Function used in CSC variant
bool LpSpRp(ReedsSheppNode node, double & t, double & u, double & v)
{
  double t1, u1;
  toPolarCoordinates(node.x + std::sin(node.phi), node.y - 1.0 - std::cos(node.phi), u1, t1);
  u1 = std::pow(u1, 2.0);
  if (u1 >= 4.0) {
    double theta;
    u = std::sqrt(u1 - 4.);
    theta = std::atan2(2.0, u);
    t = autoware::common::helper_functions::wrap_angle(t1 + theta);
    v = autoware::common::helper_functions::wrap_angle(t - node.phi);

    assert(std::abs(2.0 * std::sin(t) + u * std::cos(t) - std::sin(node.phi) - node.x) < EPSILON);
    assert(
      std::abs(-2.0 * std::cos(t) + u * std::sin(t) + std::cos(node.phi) + 1.0 - node.y) < EPSILON);
    assert(
      std::abs(autoware::common::helper_functions::wrap_angle(t - v - node.phi)) < EPSILON);
    return t >= -NUMERIC_ZERO && v >= -NUMERIC_ZERO;
  }
  return false;
}

// formula 8.3 / 8.4 in Reeds-Shepp paper (***TYPO IN PAPER***)
/// \brief Function used in CCC variant
bool LpRmL(ReedsSheppNode node, double & t, double & u, double & v)
{
  double xi = node.x - std::sin(node.phi);
  double eta = node.y - 1. + std::cos(node.phi);
  double u1 = 0.0;
  double theta = 0.0;
  toPolarCoordinates(xi, eta, u1, theta);
  if (u1 <= 4.0) {
    u = -2.0 * std::asin(.25 * u1);
    t = autoware::common::helper_functions::wrap_angle(theta + 0.5 * u + PI);
    v = autoware::common::helper_functions::wrap_angle(node.phi - t + u);

    assert(std::abs(2.0 * (std::sin(t) - std::sin(t - u)) + std::sin(node.phi) - node.x) < EPSILON);
    assert(
      std::abs(2.0 * (-std::cos(t) + std::cos(t - u)) - std::cos(node.phi) + 1.0 - node.y) <
      EPSILON);
    assert(
      std::abs(autoware::common::helper_functions::wrap_angle(t - u + v - node.phi)) <
      EPSILON);
    return t >= -NUMERIC_ZERO && u <= NUMERIC_ZERO;
  }
  return false;
}


// formula 8.7 in Reeds-Shepp paper
/// \brief Function used in CCCC variant
bool LpRupLumRm(ReedsSheppNode node, double & t, double & u, double & v)
{
  double xi = node.x + std::sin(node.phi);
  double eta = node.y - 1.0 - std::cos(node.phi);
  double rho = 0.25 * (2.0 + std::hypot(xi, eta));
  if (rho <= 1.0) {
    u = std::acos(rho);
    calculateTauAndOmega(u, -u, xi, eta, node.phi, t, v);
    assert(
      std::abs(
        2.0 * (std::sin(t) - std::sin(t - u) + std::sin(t - 2.0 * u)) -
        std::sin(node.phi) - node.x) < EPSILON);
    assert(
      std::abs(
        2.0 * (-std::cos(t) + std::cos(t - u) - std::cos(t - 2.0 * u)) +
        std::cos(node.phi) + 1.0 - node.y) < EPSILON);
    assert(
      std::abs(
        autoware::common::helper_functions::wrap_angle(
          t - 2.0 * u - v - node.phi)) < EPSILON);
    return t >= -NUMERIC_ZERO && v <= NUMERIC_ZERO;
  }
  return false;
}

// formula 8.8 in Reeds-Shepp paper
/// \brief Function used in CCCC variant
bool LpRumLumRp(ReedsSheppNode node, double & t, double & u, double & v)
{
  double xi = node.x + std::sin(node.phi);
  double eta = node.y - 1.0 - std::cos(node.phi);
  double rho = (20.0 - std::pow(xi, 2.0) - std::pow(eta, 2.0)) / 16.0;
  if (rho >= 0.0 && rho <= 1.0) {
    u = -std::acos(rho);
    if (u >= -0.5 * PI) {
      calculateTauAndOmega(u, u, xi, eta, node.phi, t, v);
      assert(
        std::abs(4.0 * std::sin(t) - 2.0 * std::sin(t - u) - std::sin(node.phi) - node.x) <
        EPSILON);
      assert(
        std::abs(-4.0 * std::cos(t) + 2.0 * std::cos(t - u) + std::cos(node.phi) + 1.0 - node.y) <
        EPSILON);
      assert(std::abs(autoware::common::helper_functions::wrap_angle(t - v - node.phi)) < EPSILON);
      return t >= -NUMERIC_ZERO && v >= -NUMERIC_ZERO;
    }
  }
  return false;
}

//  formula 8.9 in Reeds-Shepp paper
/// \brief Function used in CCSC variant
bool LpRmSmLm(ReedsSheppNode node, double & t, double & u, double & v)
{
  double rho = 0.0;
  double theta = 0.0;
  double xi = node.x - std::sin(node.phi);
  double eta = node.y - 1.0 + std::cos(node.phi);
  toPolarCoordinates(xi, eta, rho, theta);

  if (rho >= 2.0) {
    double r = std::sqrt(rho * rho - 4.0);
    u = 2.0 - r;

    t = autoware::common::helper_functions::wrap_angle(theta + std::atan2(r, -2.0));
    v = autoware::common::helper_functions::wrap_angle(node.phi - 0.5 * PI - t);

    assert(
      std::abs(2.0 * (std::sin(t) - std::cos(t)) - u * std::sin(t) + std::sin(node.phi) - node.x) <
      EPSILON);
    assert(
      std::abs(
        -2.0 * (std::sin(t) + std::cos(t)) + u * std::cos(t) - std::cos(node.phi) + 1.0 -
        node.y) <
      EPSILON);
    assert(
      std::abs(
        autoware::common::helper_functions::wrap_angle(
          t + PI / 2.0 + v - node.phi)) < EPSILON);
    return t >= -NUMERIC_ZERO && u <= NUMERIC_ZERO && v <= NUMERIC_ZERO;
  }
  return false;
}

// formula 8.10 in Reeds-Shepp paper
/// \brief Function used in CCSC variant
bool LpRmSmRm(ReedsSheppNode node, double & t, double & u, double & v)
{
  double rho = 0.0;
  double theta = 0.0;
  double xi = node.x + std::sin(node.phi);
  double eta = node.y - 1.0 - std::cos(node.phi);
  toPolarCoordinates(-eta, xi, rho, theta);

  if (rho >= 2.0) {
    t = theta;
    u = 2.0 - rho;
    v = autoware::common::helper_functions::wrap_angle(t + 0.5 * PI - node.phi);

    assert(std::abs(2 * std::sin(t) - std::cos(t - v) - u * std::sin(t) - node.x) < EPSILON);
    assert(std::abs(-2 * std::cos(t) - std::sin(t - v) + u * std::cos(t) + 1.0 - node.y) < EPSILON);
    assert(
      std::abs(
        autoware::common::helper_functions::wrap_angle(
          t + PI / 2.0 - v - node.phi)) < EPSILON);
    return t >= -NUMERIC_ZERO && u <= NUMERIC_ZERO && v <= NUMERIC_ZERO;
  }
  return false;
}


// formula 8.11 in Reeds-Shepp paper (***TYPO IN PAPER***)
/// \brief Function used in CCSCC variant
bool LpRmSLmRp(ReedsSheppNode node, double & t, double & u, double & v)
{
  double rho = 0.0;
  double theta = 0.0;
  double xi = node.x + std::sin(node.phi);
  double eta = node.y - 1.0 - std::cos(node.phi);
  toPolarCoordinates(xi, eta, rho, theta);

  if (rho >= 2.0) {
    u = 4.0 - std::sqrt(rho * rho - 4.0);
    if (u <= NUMERIC_ZERO) {
      t =
        autoware::common::helper_functions::wrap_angle(
        std::atan2(
          (4.0 - u) * xi - 2 * eta,
          -2.0 * xi + (u - 4.0) * eta));
      v = autoware::common::helper_functions::wrap_angle(t - node.phi);
      assert(
        std::abs(
          4.0 * std::sin(t) - 2.0 * std::cos(t) - u * std::sin(t) - std::sin(node.phi) - node.x) <
        EPSILON);
      assert(
        std::abs(
          -4.0 * std::cos(t) - 2.0 * std::sin(t) + u * std::cos(t) + std::cos(node.phi) + 1.0 -
          node.y) < EPSILON);
      assert(std::abs(autoware::common::helper_functions::wrap_angle(t - v - node.phi)) < EPSILON);
      return t >= -NUMERIC_ZERO && v >= -NUMERIC_ZERO;
    }
  }
  return false;
}
}  // namespace

void CSC(ReedsSheppNode node, ReedsSheppPath & path)
{
  double L_min = path.length();
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  double L = 0.0;
  if (LpSpLp(node, t, u, v) && L_min > (L = lengthFromParameters(t, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[14], t, u, v);
    L_min = L;
  }
  if (LpSpLp(node.timeflipped(), t, u, v) && L_min > (L = lengthFromParameters(t, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[14], -t, -u, -v);
    L_min = L;
  }
  if (LpSpLp(node.reflected(), t, u, v) && L_min > (L = lengthFromParameters(t, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[15], t, u, v);
    L_min = L;
  }
  if (
    LpSpLp(node.timeflipped().reflected(), t, u, v) &&
    L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[15], -t, -u, -v);
    L_min = L;
  }
  if (LpSpRp(node, t, u, v) && L_min > (L = lengthFromParameters(t, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[12], t, u, v);
    L_min = L;
  }
  if (LpSpRp(node.timeflipped(), t, u, v) && L_min > (L = lengthFromParameters(t, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[12], -t, -u, -v);
    L_min = L;
  }
  if (LpSpRp(node.reflected(), t, u, v) && L_min > (L = lengthFromParameters(t, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[13], t, u, v);
    L_min = L;
  }
  if (
    LpSpRp(node.timeflipped().reflected(), t, u, v) &&
    L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[13], -t, -u, -v);
  }
}

void CCC(ReedsSheppNode node, ReedsSheppPath & path)
{
  double L_min = path.length();
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  double L = 0.0;
  if (LpRmL(node, t, u, v) && L_min > (L = lengthFromParameters(t, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[0], t, u, v);
    L_min = L;
  }
  if (LpRmL(node.timeflipped(), t, u, v) && L_min > (L = lengthFromParameters(t, u, v))) {
    // time flip
    path = ReedsSheppPath(RP_PATH_TYPE[0], -t, -u, -v);
    L_min = L;
  }
  if (LpRmL(node.reflected(), t, u, v) && L_min > (L = lengthFromParameters(t, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[1], t, u, v);
    L_min = L;
  }
  if (
    LpRmL(node.timeflipped().reflected(), t, u, v) &&
    L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[1], -t, -u, -v);
    L_min = L;
  }

  // backwards
  ReedsSheppNode backward_node{
    node.x * std::cos(node.phi) + node.y * std::sin(node.phi),
    node.x * std::sin(node.phi) - node.y * std::cos(node.phi),
    node.phi};

  if (LpRmL(backward_node, t, u, v) && L_min > (L = lengthFromParameters(t, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[0], v, u, t);
    L_min = L;
  }
  if (
    LpRmL(backward_node.timeflipped(), t, u, v) &&
    L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[0], -v, -u, -t);
    L_min = L;
  }
  if (
    LpRmL(backward_node.reflected(), t, u, v) &&
    L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[1], v, u, t);
    L_min = L;
  }
  if (
    LpRmL(backward_node.timeflipped().reflected(), t, u, v) &&
    L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[1], -v, -u, -t);
  }
}

void CCCC(ReedsSheppNode node, ReedsSheppPath & path)
{
  double L_min = path.length();
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  double L = 0.0;
  if (LpRupLumRm(node, t, u, v) && L_min > (L = lengthFromParameters(t + 2.0, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[2], t, u, -u, v);
    L_min = L;
  }
  if (
    LpRupLumRm(node.timeflipped(), t, u, v) &&
    L_min > (L = lengthFromParameters(t + 2.0, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[2], -t, -u, u, -v);
    L_min = L;
  }
  if (
    LpRupLumRm(node.reflected(), t, u, v) &&
    L_min > (L = lengthFromParameters(t + 2.0, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[3], t, u, -u, v);
    L_min = L;
  }
  if (
    LpRupLumRm(node.timeflipped().reflected(), t, u, v) &&
    L_min > (L = lengthFromParameters(t + 2.0, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[3], -t, -u, u, -v);
    L_min = L;
  }

  if (LpRumLumRp(node, t, u, v) && L_min > (L = lengthFromParameters(t + 2.0, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[2], t, u, u, v);
    L_min = L;
  }
  if (
    LpRumLumRp(node.timeflipped(), t, u, v) &&
    L_min > (L = lengthFromParameters(t + 2.0, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[2], -t, -u, -u, -v);
    L_min = L;
  }
  if (
    LpRumLumRp(node.reflected(), t, u, v) &&
    L_min > (L = lengthFromParameters(t + 2.0, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[3], t, u, u, v);
    L_min = L;
  }
  if (
    LpRumLumRp(node.timeflipped().reflected(), t, u, v) &&
    L_min > (L = lengthFromParameters(t + 2.0, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[3], -t, -u, -u, -v);
  }
}

void CCSC(ReedsSheppNode node, ReedsSheppPath & path)
{
  double L_min = path.length() - 0.5 * PI;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  double L = 0.0;
  if (LpRmSmLm(node, t, u, v) && L_min > (L = lengthFromParameters(t, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[4], t, -0.5 * PI, u, v);
    L_min = L;
  }
  if (LpRmSmLm(node.timeflipped(), t, u, v) && L_min > (L = lengthFromParameters(t, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[4], -t, 0.5 * PI, -u, -v);
    L_min = L;
  }
  if (
    LpRmSmLm(node.reflected(), t, u, v) && L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[5], t, -0.5 * PI, u, v);
    L_min = L;
  }
  if (
    LpRmSmLm(node.timeflipped().reflected(), t, u, v) &&
    L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[5], -t, 0.5 * PI, -u, -v);
    L_min = L;
  }

  if (LpRmSmRm(node, t, u, v) && L_min > (L = lengthFromParameters(t, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[8], t, -0.5 * PI, u, v);
    L_min = L;
  }
  if (LpRmSmRm(node.timeflipped(), t, u, v) && L_min > (L = lengthFromParameters(t, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[8], -t, 0.5 * PI, -u, -v);
    L_min = L;
  }
  if (
    LpRmSmRm(node.reflected(), t, u, v) && L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[9], t, -0.5 * PI, u, v);
    L_min = L;
  }
  if (
    LpRmSmRm(node.timeflipped().reflected(), t, u, v) &&
    L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[9], -t, 0.5 * PI, -u, -v);
    L_min = L;
  }

  // backwards
  ReedsSheppNode backward_node{
    node.x * std::cos(node.phi) + node.y * std::sin(node.phi),
    node.x * std::sin(node.phi) - node.y * std::cos(node.phi),
    node.phi};

  if (LpRmSmLm(backward_node, t, u, v) && L_min > (L = lengthFromParameters(t, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[6], v, u, -0.5 * PI, t);
    L_min = L;
  }
  if (
    LpRmSmLm(backward_node.timeflipped(), t, u, v) &&
    L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[6], -v, -u, 0.5 * PI, -t);
    L_min = L;
  }
  if (
    LpRmSmLm(backward_node.reflected(), t, u, v) &&
    L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[7], v, u, -0.5 * PI, t);
    L_min = L;
  }
  if (
    LpRmSmLm(backward_node.timeflipped().reflected(), t, u, v) &&
    L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[7], -v, -u, 0.5 * PI, -t);
    L_min = L;
  }

  if (LpRmSmRm(backward_node, t, u, v) && L_min > (L = lengthFromParameters(t, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[10], v, u, -0.5 * PI, t);
    L_min = L;
  }
  if (
    LpRmSmRm(backward_node.timeflipped(), t, u, v) &&
    L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[10], -v, -u, 0.5 * PI, -t);
    L_min = L;
  }
  if (
    LpRmSmRm(backward_node.reflected(), t, u, v) &&
    L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[11], v, u, -0.5 * PI, t);
    L_min = L;
  }
  if (
    LpRmSmRm(backward_node.timeflipped().reflected(), t, u, v) &&
    L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[11], -v, -u, 0.5 * PI, -t);
  }
}

void CCSCC(ReedsSheppNode node, ReedsSheppPath & path)
{
  double L_min = path.length() - PI;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  double L = 0.0;
  if (LpRmSLmRp(node, t, u, v) && L_min > (L = lengthFromParameters(t, u, v))) {
    path = ReedsSheppPath(RP_PATH_TYPE[16], t, -0.5 * PI, u, -0.5 * PI, v);
    L_min = L;
  }
  if (
    LpRmSLmRp(node.timeflipped(), t, u, v) &&
    L_min > (L = lengthFromParameters(t, u, v)))  // time flip
  {
    path = ReedsSheppPath(RP_PATH_TYPE[16], -t, 0.5 * PI, -u, 0.5 * PI, -v);
    L_min = L;
  }
  if (
    LpRmSLmRp(node.reflected(), t, u, v) && L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[17], t, -0.5 * PI, u, -0.5 * PI, v);
    L_min = L;
  }
  if (
    LpRmSLmRp(node.timeflipped().reflected(), t, u, v) &&
    L_min > (L = lengthFromParameters(t, u, v)))
  {
    path = ReedsSheppPath(RP_PATH_TYPE[17], -t, 0.5 * PI, -u, 0.5 * PI, -v);
  }
}

}  // namespace reeds_shepp
}  // namespace freespace_planner
}  // namespace planning
}  // namespace autoware
