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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the vesc_interface class.

#ifndef VESC_INTERFACE__VESC_INTERFACE_HPP_
#define VESC_INTERFACE__VESC_INTERFACE_HPP_

#include <vesc_interface/visibility_control.hpp>

#include <common/types.hpp>
#include <vehicle_interface/platform_interface.hpp>

#include <autoware_auto_msgs/msg/high_level_control_command.hpp>
#include <autoware_auto_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_odometry.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>

#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>

#include <autoware_auto_msgs/srv/autonomy_mode_change.hpp>

#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <chrono>
#include <cstdint>

using autoware_auto_msgs::msg::RawControlCommand;
using autoware_auto_msgs::msg::VehicleControlCommand;
using autoware_auto_msgs::msg::VehicleStateCommand;
using autoware_auto_msgs::msg::VehicleStateReport;
using autoware_auto_msgs::msg::VehicleOdometry;

using vesc_msgs::msg::VescStateStamped;

using std_msgs::msg::Float64;

namespace autoware
{
/// \brief TODO(jacobjj): Document namespaces!
namespace vesc_interface
// Inherit from vehicle_interface::platform_interface
{
/// Platform interface implementation for VESC. Bridges data to and from the VESC
/// to convert speed and wheel angle position to motor speed and servo position.
/// \brief Class for interfacing with VESC
class VESC_INTERFACE_PUBLIC VESCInterface
  : public ::autoware::drivers::vehicle_interface::PlatformInterface
{
public:
  /// \brief Default Constructor.
  /// TODO(jacobjj): Comment
  VESCInterface(
    rclcpp::Node & node,
    double speed_to_erpm_gain,
    double speed_to_erpm_offset,
    double steering_to_servo_gain,
    double steering_to_servo_offset
  );

  /// \brief Sends True message
  bool8_t update(std::chrono::nanoseconds timeout);

  // send_state_command() - Most for gears, transmit the gear to VESC driver
  // Gear equivalent of reverse/ forward
  bool8_t send_state_command(const VehicleStateCommand & msg);

  // send_control_command() - desired speed and desired tire angle
  // Convert those to motorRPM, servo positions
  // RawControlCommand - Log NotSupported!!
  // IF manual mode - send zeros.
  bool8_t send_control_command(const VehicleControlCommand & msg);

  // handle_mode_change_request()
  // Switch between autonomous, and manual mode
  // maintain internal state, and only send commands when autonomous mode is active
  // IF manual mode - send zeros.
  bool8_t handle_mode_change_request(
    autoware_auto_msgs::srv::AutonomyModeChange_Request::SharedPtr request);

  /// \brief Send raw control commands, currently not implemented, hence logs error.
  bool8_t send_control_command(const RawControlCommand & msg);

  // state_report() -> Set the gear (forward/backward)

  // odometry() -> velocity_mps meters/s
  //               front_wheel_angle_rad (radians, positive-to the left)
  // https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_msgs/msg/VehicleOdometry.idl

private:
  // ROS parameters
  rclcpp::Logger m_logger;
  // conversion and gain offsets
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;

  // Direction:
  //      1: Forward
  //     -1: Reverse
  int direction{1};
  bool seen_zero_speed{false};

  // ROS services
  rclcpp::Publisher<Float64>::SharedPtr erpm_pub_;
  rclcpp::Publisher<Float64>::SharedPtr servo_pub_;
  rclcpp::Subscription<VescStateStamped>::SharedPtr vesc_motor_state_;
  rclcpp::Subscription<Float64>::SharedPtr servo_state_;

  /// \brief Receives the vesc motor state and converts it to car velocity
  /// \param[in] msg Message from the vesc for the motor status
  void on_motor_state_report(const VescStateStamped::SharedPtr & msg);

  /// \brief Receives the servo position and converts it to front wheel angle
  /// \param[in] msg Message from the vesc for the servo status
  void on_servo_state_report(const Float64::SharedPtr & msg);
};  // class VESCInterface

}  // namespace vesc_interface
}  // namespace autoware

#endif  // VESC_INTERFACE__VESC_INTERFACE_HPP_
