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
/// \brief This file defines the tracking_nodes_node class.

#ifndef TRACKING_NODES__MULTI_OBJECT_TRACKER_NODE_HPP_
#define TRACKING_NODES__MULTI_OBJECT_TRACKER_NODE_HPP_

#include <deque>

#include "autoware_auto_msgs/msg/detected_dynamic_object_array.hpp"
#include "autoware_auto_msgs/msg/tracked_dynamic_object_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tracking/multi_object_tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "synchronized_subscription/synchronized_subscription.hpp"

#include "tracking_nodes/visibility_control.hpp"

namespace autoware
{
namespace tracking_nodes
{

/// \class MultiObjectTrackerNode
/// \brief ROS 2 Node for tracking.
class TRACKING_NODES_PUBLIC MultiObjectTrackerNode : public rclcpp::Node
{
public:
  /// \brief Constructor
  explicit MultiObjectTrackerNode(const rclcpp::NodeOptions & options);

  void process(
    autoware_auto_msgs::msg::DetectedDynamicObjectArray::ConstSharedPtr objs,
    nav_msgs::msg::Odometry::ConstSharedPtr odom);
  void dropped_message_callback(
    autoware_auto_msgs::msg::DetectedDynamicObjectArray::ConstSharedPtr msg,
    autoware::common::DroppedMessageReason reason);

private:
  /// The actual tracker implementation.
  autoware::perception::tracking::MultiObjectTracker m_tracker;
  /// Subscription to odometry and detection messages .
  autoware::common::SynchronizedSubscription<autoware_auto_msgs::msg::DetectedDynamicObjectArray,
    nav_msgs::msg::Odometry> m_sub;
  /// Publisher for tracked objects.
  rclcpp::Publisher<autoware_auto_msgs::msg::TrackedDynamicObjectArray>::SharedPtr m_pub;
  rclcpp::Time m_last_objects_time = rclcpp::Time(0);
};
}  // namespace tracking_nodes
}  // namespace autoware

#endif  // TRACKING_NODES__MULTI_OBJECT_TRACKER_NODE_HPP_
