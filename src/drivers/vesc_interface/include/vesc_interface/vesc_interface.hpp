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

#include <autoware_auto_control_msgs/msg/high_level_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_vehicle_msgs/srv/autonomy_mode_change.hpp>

#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <chrono>
#include <cstdint>


using autoware::common::types::float64_t;

using autoware_auto_vehicle_msgs::msg::RawControlCommand;
using autoware_auto_vehicle_msgs::msg::VehicleControlCommand;
using autoware_auto_vehicle_msgs::msg::VehicleStateCommand;
using autoware_auto_vehicle_msgs::msg::VehicleStateReport;
using autoware_auto_vehicle_msgs::msg::VehicleOdometry;

using vesc_msgs::msg::VescStateStamped;

using std_msgs::msg::Float64;

namespace autoware
{
/// \brief A namespace to implement all functions to interface with VESC.
namespace vesc_interface
// Inherit from vehicle_interface::platform_interface
{
/// Platform interface implementation for VESC. Bridges data to and from the VESC
/// to convert speed and wheel angle position to motor speed and servo position.
/// ERPM (electric RPM) = speed_to_erpm_gain * speed (meters/second) + speed_to_erpm_offset
/// servo value (0 to 1) = steering_to_servo_gain * steering_angle (rad) + steering_to_servo_offset
/// \brief Class for interfacing with VESC
class VESC_INTERFACE_PUBLIC VESCInterface
  : public ::autoware::drivers::vehicle_interface::PlatformInterface
{
public:
  /// \brief Default Constructor.
  /// \param[in] node Reference to node
  /// \param[in] speed_to_erpm_gain Gain to convert speed(m/s) to ERPM
  /// \param[in] speed_to_erpm_offset Offset ERPM motor speed.
  /// \param[in] steering_to_servo_gain Gain to convert steering angle (rad) to servo position
  /// \param[in] steering_to_servo_offset Default servo position when car is moving straight
  VESCInterface(
    rclcpp::Node & node,
    float64_t speed_to_erpm_gain,
    float64_t speed_to_erpm_offset,
    float64_t steering_to_servo_gain,
    float64_t steering_to_servo_offset
  );

  /// \brief Sends True message
  /// \param[in] timeout The maximum amount of time to check/receive data
  /// \returns always returns True.
  bool8_t update(std::chrono::nanoseconds timeout);

  /// \brief Switch between reverse and forward direction.
  /// \param[in] msg The state command to send to the vehicle
  /// \return False if message is not send, otherwise True
  bool8_t send_state_command(const VehicleStateCommand & msg);

  /// \brief Get vehicle speed (meters/second) and steering angle (rad),
  ///        and publish motor ERPM and servo position.
  /// \param[in] msg The control command send to the vehicle
  /// \return False if message is not send, otherwise True
  bool8_t send_control_command(const VehicleControlCommand & msg);

  /// \brief Switch between autonomous and manual driving mode,
  ///        maintain internal state, and only send control commands
  ///        when autonomous mode is active
  /// \param[in] request Request object
  /// \return False if message is not send, otherwise True
  bool8_t handle_mode_change_request(
    autoware_auto_vehicle_msgs::srv::AutonomyModeChange_Request::SharedPtr request);

  /// \brief Send raw control commands, currently not implemented, hence logs error.
  /// \param[in] msg The raw message command.
  /// \return False always.
  bool8_t send_control_command(const RawControlCommand & msg);

  /// \brief Send ackermann control commands, currently not implemented, hence logs error.
  /// \param[in] msg The ackermann message command.
  /// \return False always.
  bool8_t send_control_command(const AckermannControlCommand & msg);

  // state_report() -> Set the gear (forward/backward)

private:
  // ROS parameters
  rclcpp::Logger m_logger;
  // conversion and gain offsets
  float64_t speed_to_erpm_gain_, speed_to_erpm_offset_;
  float64_t steering_to_servo_gain_, steering_to_servo_offset_;
  Float64::SharedPtr last_servo_cmd;

  // Direction:
  //      1: Forward
  //     -1: Reverse
  int32_t direction{1};
  bool8_t seen_zero_speed{false};
  bool8_t run_autonomous{false};

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
