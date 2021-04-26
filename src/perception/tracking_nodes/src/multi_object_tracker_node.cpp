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
using autoware_auto_msgs::msg::TrackedObjectArray;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;

namespace
{
// Helper function
MultiObjectTracker init_tracker(rclcpp::Node & node)
{
  MultiObjectTrackerOptions options{};
  (void) node;
  return MultiObjectTracker{options};
}

}  // namespace

MultiObjectTrackerNode::MultiObjectTrackerNode(const rclcpp::NodeOptions & options)
:  Node("multi_object_tracker_node", options),
  m_tracker(init_tracker(*this)),
  m_odometry_sub(this->create_subscription<Odometry>(
      "odometry", rclcpp::QoS(10), std::bind(&MultiObjectTrackerNode::odom_callback, this, _1))),
  m_detections_sub(this->create_subscription<DetectedDynamicObjectArray>(
      "detected_objects",
      rclcpp::QoS(10), std::bind(&MultiObjectTrackerNode::objects_callback, this, _1))),
  m_pub(this->create_publisher<TrackedObjectArray>("tracked_objects", rclcpp::QoS(10)))
{
}

void objects_callback(DetectedDynamicObjectArray::ConstSharedPtr msg)
{
  // Assume that state estimator and perception are synchronized, see #1040.
  // Also assume in-order message delivery.
  if (m_odometry_buffer.empty()) {
    // We'll need to wait for the odometry message to arrive.
    // Queue for later processing.
    if (m_objects_buffer) {
      // This will likely happen at startup.
      RCLCPP_WARN(get_logger(), "Dropping detection message since no odometry was received.");
    }
    m_objects_buffer = msg;
    return;
  }
  const auto corresponding_odom_msg = std::find_if(
    m_odometry_buffer.cbegin(), m_odometry_buffer.cend(), [&msg](const auto & odom_msg) {
      return odom_msg->header.stamp == msg->header.stamp;
    });
  if (corresponding_odom_msg != m_odometry_buffer.cend()) {
    // Both messages are here.
    this->process(msg, corresponding_odom_msg);
    // We should delete all odom messages older than msg.
    m_odometry_buffer.erase(corresponding_odometry_msg, m_odometry_buffer.cend());
  } else {
    const auto newest_odom_msg = m_odometry_buffer.front();
    if (rclcpp::Time(newest_odom_msg->header.stamp) > rclcpp::Time(msg->header.stamp)) {
      RCLCPP_WARN(get_logger(), "Dropping detection message since no odometry was received.");
    } else {
      m_objects_buffer = msg;
    }
  }
}

void odom_callback(Odometry::ConstSharedPtr msg)
{
  m_odometry_buffer.push_front(msg);
  if (!m_odometry_buffer) {
    return;
  }
}


void process(DetectedDynamicObjectArray::ConstSharedPtr objs, Odometry::ConstSharedPtr odom)
{
  const auto result = m_tracker.update(*objs, *odom);
  if (result.status == TrackerUpdateStatus::Ok) {
    m_pub.publish(std::move(result.objects));
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

}  // namespace tracking_nodes
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tracking_nodes::MultiObjectTrackerNode)
