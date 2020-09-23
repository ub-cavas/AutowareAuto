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

#include <automotive_platform_msgs/msg/gear.hpp>
#include <rclcpp/logging.hpp>
#include <time_utils/time_utils.hpp>

#include <stdexcept>

using SscGear = automotive_platform_msgs::msg::Gear;

namespace ssc_interface
{

DbwStateMachine::DbwStateMachine(uint16_t dbw_disabled_debounce)
: m_first_control_cmd_sent{false},
  m_first_state_cmd_sent{false},
  m_disabled_feedback_count{0},
  DISABLED_FEEDBACK_THRESH{dbw_disabled_debounce},
  m_state{DbwState::DISABLED}
{
}

bool8_t DbwStateMachine::enabled() const
{
  return m_state == DbwState::ENABLED ||
         m_state == DbwState::ENABLE_SENT ||
         (m_state == DbwState::ENABLE_REQUESTED &&
         m_first_control_cmd_sent &&
         m_first_state_cmd_sent);
}

DbwState DbwStateMachine::get_state() const {return m_state;}

void DbwStateMachine::dbw_feedback(bool8_t enabled)
{
  if (enabled) {                             // DBW system says enabled
    if (m_state == DbwState::ENABLE_SENT) {  // and state is ENABLE_SENT
      m_state = DbwState::ENABLED;
      m_disabled_feedback_count = 0;
    }
  } else {                                   // DBW system says disabled
    if (m_state == DbwState::ENABLE_SENT) {  // and state is ENABLE_SENT
      m_disabled_feedback_count++;           // Increase debounce count

      if (m_disabled_feedback_count > DISABLED_FEEDBACK_THRESH) {  // check debounce
        disable_and_reset();
      }
    } else if (m_state == DbwState::ENABLED) {  // and state is ENABLED
      disable_and_reset();
    }
  }
}

void DbwStateMachine::control_cmd_sent()
{
  if (m_state == DbwState::ENABLE_REQUESTED &&
    m_first_control_cmd_sent &&
    m_first_state_cmd_sent)
  {
    // We just sent a control command with
    // enable == true so we can transition
    // to ENABLE_SENT
    m_state = DbwState::ENABLE_SENT;
  }

  if (m_state == DbwState::ENABLE_REQUESTED) {
    m_first_control_cmd_sent = true;
  }
}

void DbwStateMachine::state_cmd_sent()
{
  if (m_state == DbwState::ENABLE_REQUESTED &&
    m_first_control_cmd_sent &&
    m_first_state_cmd_sent)
  {
    // We just sent a state command with
    // enable == true so we can transition
    // to ENABLE_SENT
    m_state = DbwState::ENABLE_SENT;
  }

  if (m_state == DbwState::ENABLE_REQUESTED) {
    m_first_state_cmd_sent = true;
  }
}

void DbwStateMachine::user_request(bool8_t enable)
{
  if (enable) {                           // Enable is being requested
    if (m_state == DbwState::DISABLED) {  // Only change states if currently in DISABLED
      m_state = DbwState::ENABLE_REQUESTED;
    }
  } else {                               // Disable is being requested
    if (m_state == DbwState::ENABLED) {  // Only change states if currently in ENABLED
      disable_and_reset();
    }
  }
}

void DbwStateMachine::disable_and_reset()
{
  m_state = DbwState::DISABLED;
  m_first_control_cmd_sent = false;
  m_first_state_cmd_sent = false;
  m_disabled_feedback_count = 0;
}

SscInterface::SscInterface(
  rclcpp::Node & node,
  float32_t wheelbase_m,
  float32_t max_accel_mps2,
  float32_t max_decel_mps2,
  float32_t max_yaw_rate_rad
)
: m_logger{node.get_logger()},
  m_veh_wheelbase{wheelbase_m},
  m_accel_limit{max_accel_mps2},
  m_decel_limit{max_decel_mps2},
  m_max_yaw_rate{max_yaw_rate_rad},
  m_dbw_state_machine(new DbwStateMachine{3})
{
  // Publishers (to SSC)
  m_gear_cmd_pub = node.create_publisher<GearCommand>("gear_select", 10);
  m_speed_cmd_pub = node.create_publisher<SpeedMode>("arbitrated_speed_commands", 10);
  m_steer_cmd_pub = node.create_publisher<SteerMode>("arbitrated_steering_commands", 10);
  m_turn_signal_cmd_pub = node.create_publisher<TurnSignalCommand>(
    "turn_signal_command", 10);

  // Publishers (to Autoware)
  m_kinematic_state_pub =
    node.create_publisher<VehicleKinematicState>("vehicle_kinematic_state", 10);

  // Subscribers (from SSC)
  m_dbw_state_sub =
    node.create_subscription<std_msgs::msg::Bool>(
    "dbw_enabled_feedback", rclcpp::QoS{10},
    [this](std_msgs::msg::Bool::SharedPtr msg) {on_dbw_state_report(msg);});
  m_gear_feedback_sub =
    node.create_subscription<GearFeedback>(
    "gear_feedback", rclcpp::QoS{10},
    [this](GearFeedback::SharedPtr msg) {on_gear_report(msg);});
  m_vel_accel_sub =
    node.create_subscription<VelocityAccelCov>(
    "velocity_accel_cov", rclcpp::QoS{10},
    [this](VelocityAccelCov::SharedPtr msg) {on_vel_accel_report(msg);});
}

bool8_t SscInterface::update(std::chrono::nanoseconds timeout)
{
  (void)timeout;
  return true;
}

bool8_t SscInterface::send_state_command(const VehicleStateCommand & msg)
{
  m_dbw_state_machine->user_request(msg.mode == VehicleStateCommand::MODE_AUTONOMOUS);

  // Turn signal command
  TurnSignalCommand tsc;
  tsc.mode = m_dbw_state_machine->enabled() ? 1 : 0;
  tsc.turn_signal = TurnSignalCommand::NONE;

  switch (msg.blinker) {
    case VehicleStateCommand::BLINKER_NO_COMMAND:
      break;
    case VehicleStateCommand::BLINKER_OFF:
      tsc.mode = 1;
      tsc.turn_signal = TurnSignalCommand::NONE;
      break;
    case VehicleStateCommand::BLINKER_LEFT:
      tsc.mode = 1;
      tsc.turn_signal = TurnSignalCommand::LEFT;
      break;
    case VehicleStateCommand::BLINKER_RIGHT:
      tsc.mode = 1;
      tsc.turn_signal = TurnSignalCommand::RIGHT;
      break;
    case VehicleStateCommand::BLINKER_HAZARD:
      tsc.mode = 1;
      tsc.turn_signal = TurnSignalCommand::NONE;
      RCLCPP_WARN(m_logger, "Received command for unsuported turn signal state.");
      break;
    default:
      RCLCPP_ERROR(m_logger, "Received command for invalid turn signal state.");
  }

  tsc.header.stamp = msg.stamp;
  m_turn_signal_cmd_pub->publish(tsc);

  // Gear command
  GearCommand gc;
  // Has no mode - only listens if at least one
  // other DBW system is enabled
  gc.command.gear = SscGear::NONE;

  switch (msg.gear) {
    case VehicleStateCommand::GEAR_NO_COMMAND:
      break;
    case VehicleStateCommand::GEAR_DRIVE:
      gc.command.gear = SscGear::DRIVE;
      break;
    case VehicleStateCommand::GEAR_REVERSE:
      gc.command.gear = SscGear::REVERSE;
      break;
    case VehicleStateCommand::GEAR_PARK:
      gc.command.gear = SscGear::PARK;
      break;
    case VehicleStateCommand::GEAR_LOW:
      gc.command.gear = SscGear::LOW;
      break;
    case VehicleStateCommand::GEAR_NEUTRAL:
      gc.command.gear = SscGear::NEUTRAL;
      break;
    default:
      RCLCPP_ERROR(m_logger, "Received command for invalid gear state.");
  }

  gc.header.stamp = msg.stamp;
  m_gear_cmd_pub->publish(gc);

  m_dbw_state_machine->state_cmd_sent();

  return true;
}

bool8_t SscInterface::send_control_command(const HighLevelControlCommand & msg)
{
  // Publish speed command
  SpeedMode speed_mode;
  speed_mode.mode = m_dbw_state_machine->enabled() ? 1 : 0;
  speed_mode.speed = msg.velocity_mps;
  speed_mode.acceleration_limit = m_accel_limit;
  speed_mode.deceleration_limit = m_decel_limit;
  speed_mode.header.stamp = msg.stamp;
  m_speed_cmd_pub->publish(speed_mode);

  // Publish steering command
  SteerMode steer_mode;
  steer_mode.mode = m_dbw_state_machine->enabled() ? 1 : 0;
  steer_mode.curvature = msg.curvature;
  steer_mode.max_curvature_rate = yaw_rate_to_curvature_rate(m_max_yaw_rate, msg.velocity_mps);
  steer_mode.header.stamp = msg.stamp;
  m_steer_cmd_pub->publish(steer_mode);

  m_dbw_state_machine->control_cmd_sent();

  return true;
}

bool8_t SscInterface::send_control_command(const RawControlCommand & msg)
{
  (void)msg;
  RCLCPP_ERROR(m_logger, "SSC does not support sending raw pedal controls directly.");
  return false;
}

bool8_t SscInterface::send_control_command(const VehicleControlCommand & msg)
{
  // const auto directional_accel = get_state_report().gear ==
  //   VehicleStateReport::GEAR_REVERSE ? -msg.long_accel_mps2 : msg.long_accel_mps2;

  HighLevelControlCommand hlc_cmd;
  hlc_cmd.stamp = msg.stamp;

  // TODO(j.whitley) Calculate desired speed from acceleration
  hlc_cmd.velocity_mps = 0.0F;

  // TODO(j.whitley) Calculate desired curvature from wheel angle using bicycle model
  hlc_cmd.curvature = 0.0F;

  return send_control_command(hlc_cmd);
}

void SscInterface::on_dbw_state_report(const std_msgs::msg::Bool::SharedPtr & msg)
{
  if (msg->data) {
    state_report().mode = VehicleStateReport::MODE_AUTONOMOUS;
  } else {
    state_report().mode = VehicleStateReport::MODE_MANUAL;
  }

  m_dbw_state_machine->dbw_feedback(msg->data);
}

void SscInterface::on_gear_report(const GearFeedback::SharedPtr & msg)
{
  switch (msg->current_gear.gear) {
    case SscGear::PARK:
      state_report().gear = VehicleStateReport::GEAR_PARK;
      break;
    case SscGear::REVERSE:
      state_report().gear = VehicleStateReport::GEAR_REVERSE;
      break;
    case SscGear::NEUTRAL:
      state_report().gear = VehicleStateReport::GEAR_NEUTRAL;
      break;
    case SscGear::DRIVE:
      state_report().gear = VehicleStateReport::GEAR_DRIVE;
      break;
    case SscGear::LOW:
      state_report().gear = VehicleStateReport::GEAR_LOW;
      break;
    case SscGear::NONE:
    default:
      state_report().gear = 0;
      RCLCPP_WARN(m_logger, "Received invalid gear value from SSC.");
  }
}

void SscInterface::on_steer_report(const SteeringFeedback::SharedPtr & msg)
{
  odometry().stamp = msg->header.stamp;
  odometry().front_wheel_angle_rad = msg->steering_wheel_angle * STEERING_TO_TIRE_RATIO;
  odometry().rear_wheel_angle_rad = 0.0F;
}

void SscInterface::on_vel_accel_report(const VelocityAccelCov::SharedPtr & msg)
{
  odometry().stamp = msg->header.stamp;
  odometry().velocity_mps = msg->velocity;
}

}  // namespace ssc_interface
