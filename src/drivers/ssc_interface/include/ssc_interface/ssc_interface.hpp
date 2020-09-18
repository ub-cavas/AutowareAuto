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

/// \copyright Copyright 2020 The Autoware Foundation
/// \file
/// \brief This file defines the ssc_interface class.

#ifndef SSC_INTERFACE__SSC_INTERFACE_HPP_
#define SSC_INTERFACE__SSC_INTERFACE_HPP_

#include <ssc_interface/visibility_control.hpp>

#include <common/types.hpp>
#include <vehicle_interface/platform_interface.hpp>

#include <automotive_platform_msgs/msg/gear_command.hpp>
#include <automotive_platform_msgs/msg/gear_feedback.hpp>
#include <automotive_platform_msgs/msg/speed_mode.hpp>
#include <automotive_platform_msgs/msg/steer_mode.hpp>
#include <automotive_platform_msgs/msg/turn_signal_command.hpp>
#include <autoware_auto_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>
#include <std_msgs/msg/bool.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>

using autoware::common::types::bool8_t;

using automotive_platform_msgs::msg::GearCommand;
using automotive_platform_msgs::msg::GearFeedback;
using automotive_platform_msgs::msg::SpeedMode;
using automotive_platform_msgs::msg::SteerMode;
using automotive_platform_msgs::msg::TurnSignalCommand;
using autoware_auto_msgs::msg::RawControlCommand;
using autoware_auto_msgs::msg::VehicleControlCommand;
using autoware_auto_msgs::msg::VehicleStateCommand;

namespace ssc_interface
{

/// \brief Class for interfacing with AS SSC
class SSC_INTERFACE_PUBLIC SscInterface
  : public ::autoware::drivers::vehicle_interface::PlatformInterface
{
public:
  explicit SscInterface(rclcpp::Node & node);
  ~SscInterface() noexcept override = default;

  /// \brief Try to receive data from the vehicle platform, and update StateReport and Odometry.
  ///   Exceptions may be thrown on errors
  /// \param[in] timeout The maximum amount of time to check/receive data
  /// \return True if data was received before the timeout, false otherwise
  bool8_t update(std::chrono::nanoseconds timeout) override;
  /// \brief Send the state command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The state command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_state_command(const VehicleStateCommand & msg) override;
  /// \brief Send the control command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_control_command(const VehicleControlCommand & msg) override;
  /// \brief Send the control command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_control_command(const RawControlCommand & msg) override;

private:
  rclcpp::Publisher<GearCommand>::SharedPtr m_gear_cmd_pub;
  rclcpp::Publisher<SpeedMode>::SharedPtr m_speed_cmd_pub;
  rclcpp::Publisher<SteerMode>::SharedPtr m_steer_cmd_pub;
  rclcpp::Publisher<TurnSignalCommand>::SharedPtr m_turn_signal_cmd_pub;
  rclcpp::SubscriptionBase::SharedPtr m_dbw_state_sub, m_gear_feedback_sub;
  rclcpp::Logger m_logger;

  void on_dbw_state_report(const std_msgs::msg::Bool::SharedPtr & msg);
  void on_gear_report(const GearFeedback::SharedPtr & msg);
};

}  // namespace ssc_interface

#endif  // SSC_INTERFACE__SSC_INTERFACE_HPP_
