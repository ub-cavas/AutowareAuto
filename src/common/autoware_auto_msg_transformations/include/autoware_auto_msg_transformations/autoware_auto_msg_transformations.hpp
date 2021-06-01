// Copyright 2021 The Autoware Foundation
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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the autoware_auto_msg_transformations class.

#ifndef AUTOWARE_AUTO_MSG_TRANSFORMATIONS__AUTOWARE_AUTO_MSG_TRANSFORMATIONS_HPP_
#define AUTOWARE_AUTO_MSG_TRANSFORMATIONS__AUTOWARE_AUTO_MSG_TRANSFORMATIONS_HPP_

#include <autoware_auto_msg_transformations/visibility_control.hpp>

#include <autoware_auto_msgs/msg/relative_position_with_covariance_stamped.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace autoware
{
namespace common
{
/// \brief Commonly used gemometric transformations for messages.
namespace msg_transformations
{

/// \brief Immutable version of the `change_frame` function.
/// \tparam TransformT The transformation type, e.g. Eigen::Isometry3f or Eigen::Isometry3d
/// \tparam MsgT The message type
template<typename TransformT, typename MsgT>
MsgT AUTOWARE_AUTO_MSG_TRANSFORMATIONS_PUBLIC change_frame_imm(
  const TransformT & tf, const MsgT & msg)
{
  MsgT out = msg;
  change_frame(tf, out);
  return out;
}

void AUTOWARE_AUTO_MSG_TRANSFORMATIONS_PUBLIC change_frame(
  const Eigen::Matrix3d & rot,
  geometry_msgs::msg::Vector3 & msg);

void AUTOWARE_AUTO_MSG_TRANSFORMATIONS_PUBLIC change_frame(
  const Eigen::Isometry3d & tf,
  geometry_msgs::msg::Point & msg);

void AUTOWARE_AUTO_MSG_TRANSFORMATIONS_PUBLIC change_frame(
  const Eigen::Isometry3d & tf,
  geometry_msgs::msg::Quaternion & msg);

void AUTOWARE_AUTO_MSG_TRANSFORMATIONS_PUBLIC change_frame(
  const Eigen::Isometry3d & tf, geometry_msgs::msg::Pose & msg);

void AUTOWARE_AUTO_MSG_TRANSFORMATIONS_PUBLIC change_frame(
  const Eigen::Isometry3d & tf, geometry_msgs::msg::PoseWithCovariance & msg);

void AUTOWARE_AUTO_MSG_TRANSFORMATIONS_PUBLIC change_frame(
  const Eigen::Isometry3d & tf, geometry_msgs::msg::TwistWithCovarianceStamped & msg);

void AUTOWARE_AUTO_MSG_TRANSFORMATIONS_PUBLIC change_frame(
  const Eigen::Isometry3d & tf,
  autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped & msg);

}  // namespace msg_transformations
}  // namespace common
}  // namespace autoware

#endif  // AUTOWARE_AUTO_MSG_TRANSFORMATIONS__AUTOWARE_AUTO_MSG_TRANSFORMATIONS_HPP_
