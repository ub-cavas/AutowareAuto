// Copyright 2021 Apex.AI, Inc.
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

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef TRACKING__PROJECTION_DEBUGGER_HPP_
#define TRACKING__PROJECTION_DEBUGGER_HPP_

#include <common/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include "visibility_control.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include <tracking/greedy_roi_associator.hpp>

namespace autoware
{
namespace perception
{
namespace tracking
{
class TRACKING_PUBLIC ProjectionDebuggerNode : public rclcpp::Node
{
public:
  using TrackedObjects = autoware_auto_msgs::msg::TrackedObjects;
  using TrackedObject = TrackedObjects::_objects_type::value_type;
  using TransformStamped = geometry_msgs::msg::TransformStamped;
  using Point32 = geometry_msgs::msg::Point32;

  explicit ProjectionDebuggerNode(const rclcpp::NodeOptions & options);

private:
  void timer_function();

  Point32 make_pt(float32_t x, float32_t y, float32_t z);

  TrackedObject from_centroid(float32_t x, float32_t y, float32_t z, std::size_t ID);

  void make_objects();

  void publish_tf();

  rclcpp::TimerBase::SharedPtr m_wall_timer;
  TrackedObjects m_objects;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr m_tf_pub;
  rclcpp::Publisher<TrackedObjects>::SharedPtr m_track_pub;
  std::unique_ptr<CameraModel> m_camera_ptr;
  geometry_msgs::msg::Transform m_camera_from_base_link;
};

}  // namespace tracking
}  // namespace perception
}  // namespace autoware


#endif  // TRACKING__PROJECTION_DEBUGGER_HPP_
