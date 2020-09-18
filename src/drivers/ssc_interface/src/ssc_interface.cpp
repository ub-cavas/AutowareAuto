// Copyright 2020 The Autoware Foundation
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

#include "ssc_interface/ssc_interface.hpp"

namespace ssc_interface
{

SscInterface::SscInterface(rclcpp::Node & node)
: m_logger{node.get_logger()}
{
  // Publishers
  m_gear_cmd_pub = node.create_publisher<GearCommand>("gear_select", 10);
  m_speed_cmd_pub = node.create_publisher<SpeedMode>("arbitrated_speed_commands", 10);
  m_steer_cmd_pub = node.create_publisher<SteerMode>("arbitrated_steering_commands", 10);
  m_turn_signal_cmd_pub = node.create_publisher<TurnSignalCommand>(
    "turn_signal_command", 10);

  // Subscribers
  m_dbw_state_sub =
    node.create_subscription<std_msgs::msg::Bool>(
    "dbw_enabled_feedback", rclcpp::QoS{10},
    [this](std_msgs::msg::Bool::SharedPtr msg) {on_dbw_state_report(msg);});
  m_gear_feedback_sub =
    node.create_subscription<GearFeedback>(
    "gear_feedback", rclcpp::QoS{10},
    [this](automotive_platform_msgs::msg::GearFeedback::SharedPtr msg)
    {on_gear_report(msg);});
}

bool8_t SscInterface::update(std::chrono::nanoseconds timeout)
{
  (void)timeout;
  return true;
}

bool8_t SscInterface::send_state_command(const VehicleStateCommand & msg)
{
  (void)msg;
  return true;
}

void SscInterface::on_dbw_state_report(const std_msgs::msg::Bool::SharedPtr & msg)
{
  (void)msg;
}

void SscInterface::on_gear_report(const GearFeedback::SharedPtr & msg)
{
  (void)msg;
}

}  // namespace ssc_interface
