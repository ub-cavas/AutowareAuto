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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "vesc_interface/vesc_interface.hpp"

#include <iostream>

namespace autoware
{
namespace vesc_interface
{
using VSC = autoware_auto_msgs::msg::VehicleStateCommand;
VESCInterface::VESCInterface(
  rclcpp::Node & node,
  double speed_to_erpm_gain,
  double speed_to_erpm_offset,
  double steering_to_servo_gain,
  double steering_to_servo_offset
)
: m_logger{node.get_logger()},
  speed_to_erpm_gain_{speed_to_erpm_gain},
  speed_to_erpm_offset_{speed_to_erpm_offset},
  steering_to_servo_gain_{steering_to_servo_gain},
  steering_to_servo_offset_{steering_to_servo_offset}
{
  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = node.create_publisher<Float64>("commands/motor/speed", 10);
  servo_pub_ = node.create_publisher<Float64>("commands/servo/position", 10);
}

bool8_t VESCInterface::update(std::chrono::nanoseconds timeout)
{
  (void)timeout;
  return true;
}

bool8_t VESCInterface::send_state_command(const VehicleStateCommand & msg)
{
  // If the GEAR is in reverse, set direction to -1
  if (msg.gear == VSC::GEAR_REVERSE) {
    direction = -1;
  }

  if (msg.gear == VSC::GEAR_DRIVE || msg.gear == VSC::GEAR_LOW) {
    direction = 1;
  }

  return true;
}

bool8_t VESCInterface::send_control_command(const VehicleControlCommand & msg)
{
  if (msg.velocity_mps == 0.0f) {
    seen_zero_speed = true;
  }

  // calc vesc electric RPM (speed)
  Float64 erpm_msg;
  if (seen_zero_speed) {
    erpm_msg.data = direction * speed_to_erpm_gain_ * static_cast<double>(msg.velocity_mps) +
      speed_to_erpm_offset_;
  } else {
    erpm_msg.data = 0.0f;
  }

  // calc steering angle (servo)
  Float64 servo_msg;
  servo_msg.data = direction * steering_to_servo_gain_ *
    static_cast<double>(msg.front_wheel_angle_rad) + steering_to_servo_offset_;

  if (rclcpp::ok()) {
    erpm_pub_->publish(erpm_msg);
    servo_pub_->publish(servo_msg);
    return true;
  }
  return false;
}

/// TODO(jacobjj): Add comments
bool8_t VESCInterface::handle_mode_change_request(
  autoware_auto_msgs::srv::AutonomyModeChange_Request::SharedPtr request)
{
  (void)request;
  return true;
}

bool8_t VESCInterface::send_control_command(const RawControlCommand & msg)
{
  (void)msg;
  // Log Error, Not Implemented.
  RCLCPP_WARN(m_logger, "Cannot control the VESC using RawControlCommand");
  return true;
}
}  // namespace vesc_interface
}  // namespace autoware
