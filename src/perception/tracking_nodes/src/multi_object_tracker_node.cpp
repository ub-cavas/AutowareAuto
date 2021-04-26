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

#include "tracking_nodes/multi_object_tracker_node.hpp"

#include <functional>
#include <utility>

namespace autoware
{
namespace tracking_nodes
{

using autoware::perception::tracking::MultiObjectTracker;
using autoware::perception::tracking::MultiObjectTrackerOptions;
using autoware_auto_msgs::msg::DetectedDynamicObjectArray;
using autoware_auto_msgs::msg::TrackedDynamicObjectArray;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
// Helper function
MultiObjectTracker init_tracker(rclcpp::Node & node)
{
  MultiObjectTrackerOptions options{};
  (void) node;
  return MultiObjectTracker{options};
}

auto init_subscription(rclcpp::Node & node)
{
  autoware::common::SynchronizedSubscriptionConfig<DetectedDynamicObjectArray, Odometry> config;
  config.cb = std::bind(&MultiObjectTrackerNode::process, node, _1, _2);
  config.dropped_msg_cb =
    std::bind(&MultiObjectTrackerNode::dropped_message_callback, node, _1, _2);
  return autoware::common::SynchronizedSubscription(
    "detected_objects", rclcpp::QoS(10), "odometry", rclcpp::QoS(10),
    "odometry", rclcpp::QoS(10), std::move(config));
}

}  // namespace

MultiObjectTrackerNode::MultiObjectTrackerNode(const rclcpp::NodeOptions & options)
:  Node("multi_object_tracker_node", options),
  m_tracker(init_tracker(*this)),
  m_sub(init_subscription(*this)),
  m_pub(this->create_publisher<TrackedDynamicObjectArray>("tracked_objects", rclcpp::QoS(10)))
{
}

void MultiObjectTrackerNode::process(
  DetectedDynamicObjectArray::ConstSharedPtr objs,
  Odometry::ConstSharedPtr odom)
{
  const auto result = m_tracker.update(*objs, *odom);
  if (result.status == TrackerUpdateStatus::Ok) {
    m_pub->publish(std::move(result.objects));
  } else {
    // We use a switch statement without default since it warns when not all cases are handled.
    const auto get_status_string = [](TrackerUpdateStatus status) {
        switch (result.status) {
          case TrackerUpdateStatus::Ok: return "Ok";
        }
        return "Invalid status";
      }
      RCLCPP_WARN(
      get_logger(), "Tracker update for detection at time failed. Reason: %s",
      get_status_string());
  }
}

void MultiObjectTrackerNode::dropped_message_callback(
  autoware_auto_msgs::msg::DetectedDynamicObjectArray::ConstSharedPtr msg,
  autoware::common::DroppedMessageReason reason)
{
  RCLCPP_WARN(get_logger(), "Dropping detection message since no odometry was received.");
}

}  // namespace tracking_nodes
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tracking_nodes::MultiObjectTrackerNode)
