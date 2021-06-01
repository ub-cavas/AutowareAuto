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

#include "autoware_auto_msg_transformations/autoware_auto_msg_transformations.hpp"

#include <iostream>

namespace autoware
{
namespace common
{
namespace msg_transformations
{

void change_frame(const Eigen::Matrix3d & rot, geometry_msgs::msg::Vector3 & msg)
{
  Eigen::Vector3d p{msg.x, msg.y, msg.z};
  p = rot * p;
  msg.x = p.x();
  msg.y = p.y();
  msg.z = p.z();
}

void change_frame(const Eigen::Isometry3d & tf, geometry_msgs::msg::Point & msg)
{
  Eigen::Vector3d p{msg.x, msg.y, msg.z};
  p = tf * p;
  msg.x = p.x();
  msg.y = p.y();
  msg.z = p.z();
}

void change_frame(const Eigen::Isometry3d & tf, geometry_msgs::msg::Quaternion & msg)
{
  const Eigen::Quaterniond q_orig{msg.w, msg.x, msg.y, msg.z};
  const Eigen::Quaterniond q_tf(tf.linear());
  const Eigen::Quaterniond q = q_tf * q_orig;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
}

void change_frame(const Eigen::Isometry3d & tf, geometry_msgs::msg::Pose & msg)
{
  change_frame(tf, msg.position);
  change_frame(tf, msg.orientation);
}

void change_frame(const Eigen::Isometry3d & tf, geometry_msgs::msg::PoseWithCovariance & msg)
{
  change_frame(tf, msg.pose);
  // We assume a Gaussian distribution.
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> cov(msg.covariance.data());
  const Eigen::Matrix3d pos_cov_transformed = tf.linear() *
    cov.topLeftCorner<3, 3>() * tf.linear().transpose();
  cov.topLeftCorner<3, 3>() = pos_cov_transformed;
  // The non-position covariances are left alone for now,
  // since it's not clear to me what the correct formula is.
}

void change_frame(
  const Eigen::Matrix3d & rot, geometry_msgs::msg::Twist & msg)
{
  change_frame(rot, msg.linear);
  change_frame(rot, msg.angular);
}

void change_frame(
  const Eigen::Matrix3d & tf, geometry_msgs::msg::TwistWithCovariance & msg)
{
  change_frame(rot, msg.twist);
  // TODO(nikolai.morin): Add covariance transformation code.
}

void change_frame(
  const Eigen::Isometry3d & tf,
  autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped & msg)
{
  Eigen::Vector3d p{msg.position.x, msg.position.y, msg.position.z};
  p = tf * p;
  msg.position.x = p.x();
  msg.position.y = p.y();
  msg.position.z = p.z();
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> cov(msg.covariance.data());
  cov = tf.linear() * cov * tf.linear().transpose();
}


}  // namespace msg_transformations
}  // namespace common
}  // namespace autoware
