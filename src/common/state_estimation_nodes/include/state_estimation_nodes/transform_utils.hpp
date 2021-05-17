// Copyright 2021 the Autoware Foundation
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

/// \copyright Copyright 2021 the Autoware Foundation
/// All rights reserved.
/// \file
/// \brief Description.

#ifndef STATE_ESTIMATION_NODES__TRANSFORM_UTILS_HPP_
#define STATE_ESTIMATION_NODES__TRANSFORM_UTILS_HPP_


#include <autoware_auto_msgs/msg/relative_position_with_covariance_stamped.hpp>
#include <state_estimation_nodes/visibility_control.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/buffer_core.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace autoware
{
namespace common
{
namespace state_estimation
{

///
/// @brief      Gets the transformation from target to source.
///
/// @details    A transformation from target to source represents a transformation
///             `tf__target__source` such as it can be used to transform a point in the source frame
///             `p_source` to a point in the target frame `p_target` as follows: `p_target =
///             tf__target__source * p_source`.
///
/// @param[in]  buffer           The tf buffer object
/// @param[in]  target_frame_id  The target frame name
/// @param[in]  source_frame_id  The source frame name
/// @param[in]  timestamp        The timestamp at which to get the transformation
///
/// @return     The transformation from source frame to the target frame.
///
geometry_msgs::msg::TransformStamped STATE_ESTIMATION_NODES_PUBLIC get_tf__target__source(
  const tf2::BufferCore & buffer,
  const std_msgs::msg::Header::_frame_id_type & target_frame_id,
  const std_msgs::msg::Header::_frame_id_type & source_frame_id,
  const std_msgs::msg::Header::_stamp_type & timestamp);

///
/// @brief      Switch the frame_id and the child_frame_id of the message changing the message
///             accordingly
///
/// @details    This is the default catch-all implementation for any message. It must be specialized
///             for a particular message or message class.
///
/// @tparam     MsgT       Type of the message.
///
/// @return     The changed message with new frames.
///
template<typename MsgT>
MsgT STATE_ESTIMATION_NODES_PUBLIC switch_frames(
  const MsgT &,
  const std_msgs::msg::Header::_frame_id_type &,
  const std_msgs::msg::Header::_frame_id_type &,
  const tf2::BufferCore &)
{
  static_assert(sizeof(MsgT) == 0, "This method must be explicitly specified for a message type.");
}

///
/// @brief      Switch the frame_id of the message changing the message accordingly
///
/// @details    This is the default catch-all implementation for any message that does not have a
///             child_frame_id. It must be specialized for a particular message or message class.
///
/// @tparam     MsgT       Type of the message.
///
/// @return     The changed message with new frames.
///
template<typename MsgT>
MsgT STATE_ESTIMATION_NODES_PUBLIC switch_frames(
  const MsgT &,
  const std_msgs::msg::Header::_frame_id_type &,
  const tf2::BufferCore &)
{
  static_assert(sizeof(MsgT) == 0, "This method must be explicitly specified for a message type.");
}

///
/// @brief      Switch frames of the odometry message.
///
/// @warning    It is assumed that the old and the new `child_frame_id` have a rigid constant
///             transformation between them, i.e., this transformation does not change with time,
///             thus there is no relative speed between these transformations. If this is not the
///             case, the update of the twist will be wrong.
///
/// @param[in]  original_msg        The original odometry message
/// @param[in]  new_frame_id        The new frame identifier
/// @param[in]  new_child_frame_id  The new child frame identifier
/// @param[in]  buffer              The tf buffer
///
/// @return     A new odometry message with the requested frame_id and child_frame_id fields.
///
template<>
nav_msgs::msg::Odometry STATE_ESTIMATION_NODES_PUBLIC switch_frames(
  const nav_msgs::msg::Odometry & original_msg,
  const std_msgs::msg::Header::_frame_id_type & new_frame_id,
  const std_msgs::msg::Header::_frame_id_type & new_child_frame_id,
  const tf2::BufferCore & buffer);

///
/// @brief      Switch the frames for a relative position message.
///
/// @param[in]  original_msg        The original odometry message
/// @param[in]  new_frame_id        The new frame identifier
/// @param[in]  new_child_frame_id  The new child frame identifier
/// @param[in]  buffer              The tf buffer
///
/// @return     The changed relative position message with new frames
///
template<>
autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped STATE_ESTIMATION_NODES_PUBLIC
switch_frames(
  const autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped & original_msg,
  const std_msgs::msg::Header::_frame_id_type & new_frame_id,
  const std_msgs::msg::Header::_frame_id_type & new_child_frame_id,
  const tf2::BufferCore & buffer);

///
/// @brief      Switch the frame_id of the pose message.
///
/// @note       There is no way to change the child_frame_id as this message has none.
///
/// @param[in]  original_msg  The original message
/// @param[in]  new_frame_id  The new frame_id
/// @param[in]  buffer        The tf buffer
///
/// @return     The changed message with its frame_id changed.
///
template<>
geometry_msgs::msg::PoseWithCovarianceStamped STATE_ESTIMATION_NODES_PUBLIC switch_frames(
  const geometry_msgs::msg::PoseWithCovarianceStamped & original_msg,
  const std_msgs::msg::Header::_frame_id_type & new_frame_id,
  const tf2::BufferCore & buffer);

///
/// @brief      Switch the frame_id of the twist message.
///
/// @note       There is no way to change the child_frame_id as this message has none.
///
/// @param[in]  original_msg  The original message
/// @param[in]  new_frame_id  The new frame_id
/// @param[in]  buffer        The tf buffer
///
/// @return     The changed message with its frame_id changed.
///
template<>
geometry_msgs::msg::TwistWithCovarianceStamped STATE_ESTIMATION_NODES_PUBLIC switch_frames(
  const geometry_msgs::msg::TwistWithCovarianceStamped & original_msg,
  const std_msgs::msg::Header::_frame_id_type & new_frame_id,
  const tf2::BufferCore & buffer);


}  // namespace state_estimation
}  // namespace common
}  // namespace autoware


#endif  // STATE_ESTIMATION_NODES__TRANSFORM_UTILS_HPP_
