// Copyright 2019-2020 Apex.AI, Inc.
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


#ifndef LOCALIZATION_COMMON__INITIALIZATION_HPP_
#define LOCALIZATION_COMMON__INITIALIZATION_HPP_

#include <localization_common/visibility_control.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <helper_functions/crtp.hpp>
#include <tf2/buffer_core.h>
#include <time_utils/time_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
// probably include the motion model

namespace autoware
{
namespace localization
{
namespace localization_common
{

/// The Pose initializer helps initialize relative localizer algorithms with an initial guess.
/// Extrapolation policy must be defined within the implementation class.
/// \tparam Derived CRTP implementation class
template<typename Derived>
class LOCALIZATION_COMMON_PUBLIC PoseInitializerBase
  : public common::helper_functions::crtp<Derived>
{
  using PoseT = geometry_msgs::msg::TransformStamped;

public:
  /// Guess the pose at a given time point. This function will look the transform up in the
  /// transform graph between the specified frames. If an external pose has been set, it takes
  /// precedence over poses in the transform graph and will be stamped with the requested time.
  /// If extrapolation is required, the behavior is determined by the implementation class.
  /// tf2 lookup may generate exceptions if the lookup fails in other ways. For details, see
  /// tf2::BufferCore class.
  /// \param tf_graph Transform graph that contains all the transforms to look up.
  /// \param time_point Time to guess the pose.
  /// \param target_frame Target frame of the transform. (i.e. "map")
  /// \param source_frame Source frame of the transform. (i.e. "base_link")
  /// \return The transform at the given time point
  PoseT guess(
    const tf2::BufferCore & tf_graph, tf2::TimePoint time_point,
    const std::string & target_frame, const std::string & source_frame)
  {
    PoseT ret;

    if (m_external_pose_available) {
      // If someone set a transform and then requests a different transform, that's an error
      if (m_external_pose.header.frame_id != target_frame ||
        m_external_pose.child_frame_id != source_frame)
      {
        throw std::runtime_error("The pose initializer's set_external_pose() "
                "and guess() methods were called with different frames.");
      }
      m_external_pose_available = false;
      m_external_pose.header.stamp = time_utils::to_message(time_point);
      return m_external_pose;
    }

    try {
      // attempt to get transform at a given point.
      ret = tf_graph.lookupTransform(target_frame, source_frame, time_point);
      // TODO(yunus.caliskan): Consider detecting too large interpolations and issuing a
      //  warning/error.
    } catch (const tf2::ExtrapolationException &) {
      ret = this->impl().extrapolate(tf_graph, time_point, target_frame, source_frame);
    }

    return ret;
  }

  /// Store a pose which will be returned in the next call to guess().
  void set_external_pose(const PoseT & external_pose)
  {
    m_external_pose_available = true;
    m_external_pose = external_pose;
  }

private:
  PoseT m_external_pose;
  bool m_external_pose_available = false;
};

/// Pose initialization implementation where the extrapolation policy is to simply
/// use the latest available transform.
class LOCALIZATION_COMMON_PUBLIC BestEffortInitializer
  : public PoseInitializerBase<BestEffortInitializer>
{
  using PoseT = geometry_msgs::msg::TransformStamped;

public:
  ///  Get the latest available transform.
  /// \param tf_graph Transform graph that contains all the transforms to look up.
  /// \param time_point Time to guess the pose.
  /// \param target_frame Target frame of the transform. (i.e. "base_link")
  /// \param source_frame Source frame of the transform. (i.e. "map")
  /// \return The transform at the given time point
  PoseT extrapolate(
    const tf2::BufferCore & tf_graph, tf2::TimePoint time_point,
    const std::string & target_frame, const std::string & source_frame);
};

}  // namespace localization_common
}  // namespace localization
}  // namespace autoware

#endif  // LOCALIZATION_COMMON__INITIALIZATION_HPP_
