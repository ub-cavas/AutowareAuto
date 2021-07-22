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

#include "test_node/test_node_node.hpp"

namespace autoware
{
namespace test_node
{

TestNodeNode::TestNodeNode(const rclcpp::NodeOptions & options)
: Node("test_node", options),
  m_pt1_sub(this, "pt1", rclcpp::QoS(10).get_rmw_qos_profile()),
  m_pt2_sub(this, "pt2", rclcpp::QoS(10).get_rmw_qos_profile()),
  m_header_cache(m_pt2_sub, 10U),
  verbose(true)
{
  m_pt1_sub.registerCallback(
    [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
      process(msg);
    });
  print_hello();
}

void TestNodeNode::process(geometry_msgs::msg::PointStamped::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Got pt1 msg. looking for pt2 msg");
  const auto matched_msgs = m_header_cache.getInterval(msg->header.stamp, msg->header.stamp);
  if (matched_msgs.empty()) {
    RCLCPP_INFO(get_logger(), "Did not get any match");
  } else {
    RCLCPP_INFO(get_logger(), "Got match");
  }
}

}  // namespace test_node
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::test_node::TestNodeNode)
