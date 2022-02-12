#include "dataspeed_ford_interface/dataspeed_ford_interface.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

#include <rclcpp/logging.hpp>
#include <time_utils/time_utils.hpp>

namespace autoware
{
namespace dataspeed_ford_interface
{

DataspeedFordInterface::DataspeedFordInterface(
  rclcpp::Node & node,
  uint16_t ecu_build_num,
  float32_t front_axle_to_cog,
  float32_t rear_axle_to_cog,
  float32_t steer_to_tire_ratio,
  float32_t max_steer_angle,
  float32_t acceleration_limit,
  float32_t deceleration_limit,
  float32_t acceleration_positive_jerk_limit,
  float32_t deceleration_negative_jerk_limit,
  uint32_t pub_period)
: m_logger{node.get_logger()},
  m_ecu_build_num{ecu_build_num},
  m_front_axle_to_cog{front_axle_to_cog},
  m_rear_axle_to_cog{rear_axle_to_cog},
  m_steer_to_tire_ratio{steer_to_tire_ratio},
  m_max_steer_angle{max_steer_angle},
  m_acceleration_limit{acceleration_limit},
  m_deceleration_limit{std::fabs(deceleration_limit)},
  m_acceleration_positive_jerk_limit{acceleration_positive_jerk_limit},
  m_deceleration_negative_jerk_limit{deceleration_negative_jerk_limit},
  m_pub_period{std::chrono::milliseconds(pub_period)},
  m_dbw_state_machine(new DbwStateMachine{3}),
  m_clock{RCL_SYSTEM_TIME}
{
  // Publishers (to Dataspeed Fords DBW)
  m_throttle_cmd_pub = node.create_publisher<ThrottleCmd>("throttle_cmd", 1);
  m_brake_cmd_pub = node.create_publisher<BrakeCmd>("brake_cmd", 1);
  m_gear_cmd_pub = node.create_publisher<GearCmd>("gear_cmd", 1);
  m_misc_cmd_pub = node.create_publisher<MiscCmd>("misc_cmd", 1);
  m_steer_cmd_pub = node.create_publisher<SteeringCmd>("steering_cmd", 1);
  m_dbw_enable_cmd_pub = node.create_publisher<std_msgs::msg::Empty>("enable", 10);
  m_dbw_disable_cmd_pub = node.create_publisher<std_msgs::msg::Empty>("disable", 10);

  // Publishers (to Autoware)
  m_vehicle_kin_state_pub =
    node.create_publisher<VehicleKinematicState>("vehicle_kinematic_state", 10);

  // Subscribers (from Dataspeed Fords DBW)
  m_brake_rpt_sub = node.create_subscription<BrakeReport>(
    "brake_report", rclcpp::QoS{20}, [this](BrakeReport::SharedPtr msg) { on_brake_report(msg); });
  m_gear_rpt_sub = node.create_subscription<GearReport>(
    "gear_report", rclcpp::QoS{20}, [this](GearReport::SharedPtr msg) { on_gear_report(msg); });
  m_misc_rpt_sub = node.create_subscription<Misc1Report>(
    "misc_report", rclcpp::QoS{2}, [this](Misc1Report::SharedPtr msg) { on_misc_report(msg); });
  m_steering_rpt_sub = node.create_subscription<SteeringReport>(
    "steering_report", rclcpp::QoS{20}, [this](SteeringReport::SharedPtr msg) {
      on_steering_report(msg);
    });
  m_wheel_spd_rpt_sub = node.create_subscription<WheelSpeedReport>(
    "wheel_speed_report", rclcpp::QoS{20}, [this](WheelSpeedReport::SharedPtr msg) {
      on_wheel_spd_report(msg);
    });

  // Initialize command values
  m_throttle_cmd.pedal_cmd_type = ThrottleCmd::CMD_PERCENT;
  m_throttle_cmd.ignore = false;
  m_throttle_cmd.clear = false;

  m_brake_cmd.pedal_cmd_type = BrakeCmd::CMD_PERCENT;
  m_brake_cmd.ignore = false;
  m_brake_cmd.clear = false;

  m_steer_cmd.cmd_type = SteeringCmd::CMD_ANGLE;  // angular position
  m_steer_cmd.ignore = false;
  m_steer_cmd.clear = false;
  m_steer_cmd.quiet = false;
  m_steer_cmd.alert = true;
  m_max_steer_angle = SteeringCmd::ANGLE_MAX < m_max_steer_angle * DEGREES_TO_RADIANS
                        ? SteeringCmd::ANGLE_MAX
                        : m_max_steer_angle * DEGREES_TO_RADIANS;

  m_gear_cmd.cmd.gear = Gear::NONE;
  m_gear_cmd.clear = false;

  m_misc_cmd.cmd.value = TurnSignal::NONE;

  m_timer =
    node.create_wall_timer(m_pub_period, std::bind(&DataspeedFordInterface::cmdCallback, this));
}

void DataspeedFordInterface::cmdCallback()
{
  std::lock_guard<std::mutex> guard_tc(m_throttle_cmd_mutex);
  std::lock_guard<std::mutex> guard_bc(m_brake_cmd_mutex);
  std::lock_guard<std::mutex> guard_gc(m_gear_cmd_mutex);
  std::lock_guard<std::mutex> guard_mc(m_misc_cmd_mutex);
  std::lock_guard<std::mutex> guard_sc(m_steer_cmd_mutex);

  const auto is_dbw_enabled = m_dbw_state_machine->get_state() != DbwState::DISABLED;

  // Set enables based on current DBW mode
  if (is_dbw_enabled) {
    m_throttle_cmd.enable = true;
    m_brake_cmd.enable = true;
    m_steer_cmd.enable = true;
  } else {
    m_throttle_cmd.enable = false;
    m_brake_cmd.enable = false;
    m_steer_cmd.enable = false;
  }

  // Publish commands to Dataspeed Ford DBW
  m_throttle_cmd_pub->publish(m_throttle_cmd);
  m_brake_cmd_pub->publish(m_brake_cmd);
  m_gear_cmd_pub->publish(m_gear_cmd);
  m_misc_cmd_pub->publish(m_misc_cmd);
  m_steer_cmd_pub->publish(m_steer_cmd);

  // Set state flags
  m_dbw_state_machine->control_cmd_sent();
  m_dbw_state_machine->state_cmd_sent();
}

bool8_t DataspeedFordInterface::update(std::chrono::nanoseconds timeout)
{
  (void)timeout;
  return true;
}

bool8_t DataspeedFordInterface::send_state_command(const VehicleStateCommand & msg)
{
  bool8_t ret{true};

  std::lock_guard<std::mutex> guard_gc(m_gear_cmd_mutex);
  std::lock_guard<std::mutex> guard_mc(m_misc_cmd_mutex);

  // Set gear values
  switch (msg.gear) {
    case VehicleStateCommand::GEAR_NO_COMMAND:
      m_gear_cmd.cmd.gear = Gear::NONE;
      break;
    case VehicleStateCommand::GEAR_DRIVE:
      m_gear_cmd.cmd.gear = Gear::DRIVE;
      break;
    case VehicleStateCommand::GEAR_REVERSE:
      m_gear_cmd.cmd.gear = Gear::REVERSE;
      break;
    case VehicleStateCommand::GEAR_PARK:
      m_gear_cmd.cmd.gear = Gear::PARK;
      break;
    case VehicleStateCommand::GEAR_LOW:
      m_gear_cmd.cmd.gear = Gear::LOW;
      break;
    case VehicleStateCommand::GEAR_NEUTRAL:
      m_gear_cmd.cmd.gear = Gear::NEUTRAL;
      break;
    default:  // error
      m_gear_cmd.cmd.gear = Gear::NONE;
      RCLCPP_ERROR_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC, "Received command for invalid gear state.");
      ret = false;
      break;
  }

  switch (msg.blinker) {
    case VehicleStateCommand::BLINKER_NO_COMMAND:
      // Keep previous
      break;
    case VehicleStateCommand::BLINKER_OFF:
      m_misc_cmd.cmd.value = TurnSignal::NONE;
      break;
    case VehicleStateCommand::BLINKER_LEFT:
      m_misc_cmd.cmd.value = TurnSignal::LEFT;
      break;
    case VehicleStateCommand::BLINKER_RIGHT:
      m_misc_cmd.cmd.value = TurnSignal::RIGHT;
      break;
    case VehicleStateCommand::BLINKER_HAZARD:
      m_misc_cmd.cmd.value = TurnSignal::HAZARD;
      break;
    default:
      m_misc_cmd.cmd.value = TurnSignal::NONE;
      RCLCPP_ERROR_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC, "Received command for invalid turn signal state.");
      ret = false;
      break;
  }

  // set driving mode
  // FIXME (Zhihao Ruan): Should we use user_request() to handle driving mode change here?
  switch (msg.mode) {
    case VehicleStateCommand::MODE_NO_COMMAND:
      // keep previous
      break;
    case VehicleStateCommand::MODE_AUTONOMOUS:
      m_dbw_state_machine->user_request(true);
      break;
    case VehicleStateCommand::MODE_MANUAL:
      m_dbw_state_machine->user_request(false);
      break;
    default:
      RCLCPP_ERROR_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC, "Received command for invalid driving mode change");
      ret = false;
      break;
  }
  m_seen_vehicle_state_cmd = true;

  return ret;
}

/* Apparently HighLevelControlCommand will be obsolete soon.
 */
bool8_t DataspeedFordInterface::send_control_command(const HighLevelControlCommand & msg)
{
  (void)msg;
  RCLCPP_ERROR_THROTTLE(
    m_logger,
    m_clock,
    CLOCK_1_SEC,
    "Dataspeed Ford interface does not support sending high level controls directly.");
  return false;
}

/* Apparently RawControlCommand will be obsolete soon.
 * Function not supported - AutoWare RawControlCommand message units are undefined.
 */
bool8_t DataspeedFordInterface::send_control_command(const RawControlCommand & msg)
{
  (void)msg;
  RCLCPP_ERROR_THROTTLE(
    m_logger,
    m_clock,
    CLOCK_1_SEC,
    "Dataspeed Ford interface does not support sending raw pedal controls directly.");
  return false;
}

bool8_t DataspeedFordInterface::send_control_command(const VehicleControlCommand & msg)
{
  bool8_t ret{true};
  float32_t velocity_checked{0.0F};
  float32_t angle_checked{0.0F};

  std::lock_guard<std::mutex> guard_ac(m_throttle_cmd_mutex);
  std::lock_guard<std::mutex> guard_bc(m_brake_cmd_mutex);
  std::lock_guard<std::mutex> guard_sc(m_steer_cmd_mutex);

  // Check for invalid changes in direction
  if (
    ((state_report().gear == VehicleStateReport::GEAR_DRIVE) && (msg.velocity_mps < 0.0F)) ||
    ((state_report().gear == VehicleStateReport::GEAR_REVERSE) && (msg.velocity_mps > 0.0F))) {
    velocity_checked = 0.0F;
    RCLCPP_ERROR_THROTTLE(
      m_logger,
      m_clock,
      CLOCK_1_SEC,
      "Got invalid speed request value: speed direction does not match current gear.");
    ret = false;
  } else {
    velocity_checked = std::fabs(msg.velocity_mps);
  }

  // 1. Set Steering Angle
  // Limit steering angle to valid range
  /* Steering -> tire angle conversion is linear except for extreme angles */
  angle_checked = msg.front_wheel_angle_rad * m_steer_to_tire_ratio;
  if (angle_checked > m_max_steer_angle) {
    angle_checked = m_max_steer_angle;
    RCLCPP_ERROR_THROTTLE(
      m_logger,
      m_clock,
      CLOCK_1_SEC,
      "Got invalid steering angle value: request exceeds max angle.");
    ret = false;
  }
  if (angle_checked < (-1.0F * m_max_steer_angle)) {
    angle_checked = -1.0F * m_max_steer_angle;
    RCLCPP_ERROR_THROTTLE(
      m_logger,
      m_clock,
      CLOCK_1_SEC,
      "Got invalid steering angle value: request exceeds max angle.");
    ret = false;
  }
  m_steer_cmd.steering_wheel_angle_cmd = angle_checked;
  m_steer_cmd.steering_wheel_angle_velocity = 0;  // default

  // 2. Set Throttle Commands
  //  FIXME (Zhihao Ruan) Currently only use percentage values from msg.long_accel_mps2 /
  //  m_acceleration_limit as percentage of full throttle
  float32_t throttle_percent = 0, brake_percent = 0;
  if (msg.long_accel_mps2 > 0) {
    throttle_percent = msg.long_accel_mps2 / m_acceleration_limit;
    if (throttle_percent > 1.0F) {
      RCLCPP_ERROR_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC, "Received acceleration greater than m_acceleration_limit");
      throttle_percent = 1.0F;
    }
  } else {
    brake_percent = std::fabs(msg.long_accel_mps2) / m_deceleration_limit;
    if (brake_percent > 1.0F) {
      RCLCPP_ERROR_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC, "Received deceleration greater than m_acceleration_limit");
    }
    brake_percent = 1.0F;
  }
  m_throttle_cmd.pedal_cmd = throttle_percent;
  m_brake_cmd.pedal_cmd = brake_percent;

  return ret;
}

bool8_t DataspeedFordInterface::send_control_command(const AckermannControlCommand & msg)
{
  (void)msg;
  RCLCPP_ERROR_THROTTLE(
    m_logger,
    m_clock,
    CLOCK_1_SEC,
    "Dataspeed Ford interface does not support sending Ackermann controls directly.");
  return false;
}

bool8_t DataspeedFordInterface::handle_mode_change_request(ModeChangeRequest::SharedPtr request)
{
  bool8_t ret{true};
  std_msgs::msg::Empty send_req{};
  if (request->mode == ModeChangeRequest::MODE_MANUAL) {
    m_dbw_state_machine->user_request(false);
    m_dbw_disable_cmd_pub->publish(send_req);
  } else if (request->mode == ModeChangeRequest::MODE_AUTONOMOUS) {
    m_dbw_state_machine->user_request(true);
    m_dbw_enable_cmd_pub->publish(send_req);
  } else {
    RCLCPP_ERROR_THROTTLE(
      m_logger, m_clock, CLOCK_1_SEC, "Got invalid autonomy mode request value.");
    ret = false;
  }
  return ret;
}

void DataspeedFordInterface::send_headlights_command(const HeadlightsCommand & msg)
{
  (void)msg;
  RCLCPP_ERROR_THROTTLE(
    m_logger,
    m_clock,
    CLOCK_1_SEC,
    "Dataspeed Ford interface does not support sending headlights command.");
}

void DataspeedFordInterface::send_horn_command(const HornCommand & msg)
{
  (void)msg;
  RCLCPP_ERROR_THROTTLE(
    m_logger,
    m_clock,
    CLOCK_1_SEC,
    "Dataspeed Ford interface does not support sending horn command.");
}

void DataspeedFordInterface::send_wipers_command(const WipersCommand & msg)
{
  (void)msg;
  RCLCPP_ERROR_THROTTLE(
    m_logger,
    m_clock,
    CLOCK_1_SEC,
    "Dataspeed Ford interface does not support sending wipers command.");
}

void DataspeedFordInterface::on_brake_report(const BrakeReport::SharedPtr & msg)
{
  // switch (msg->parking_brake.status) {
  //   case ParkingBrake::OFF:
  //     state_report().hand_brake = false;
  //     break;
  //   case ParkingBrake::ON:
  //     state_report().hand_brake = true;
  //     break;
  //   case ParkingBrake::NO_REQUEST:
  //   case ParkingBrake::FAULT:
  //   default:
  //     state_report().hand_brake = false;
  //     RCLCPP_WARN_THROTTLE(
  //       m_logger,
  //       m_clock,
  //       CLOCK_1_SEC,
  //       "Received invalid parking brake value from Dataspeed Ford DBW.");
  //     break;
  // }
  // m_seen_brake_rpt = true;
}

void DataspeedFordInterface::on_gear_report(const GearReport::SharedPtr & msg)
{
  // switch (msg->report) {
  //   case GearReport::PARK:
  //     state_report().gear = VehicleStateReport::GEAR_PARK;
  //     break;
  //   case GearReport::REVERSE:
  //     state_report().gear = VehicleStateReport::GEAR_REVERSE;
  //     break;
  //   case GearReport::NEUTRAL:
  //     state_report().gear = VehicleStateReport::GEAR_NEUTRAL;
  //     break;
  //   case GearReport::DRIVE_1:
  //     state_report().gear = VehicleStateReport::GEAR_DRIVE;
  //     break;
  //   case GearReport::LOW:
  //     state_report().gear = VehicleStateReport::GEAR_LOW;
  //     break;
  //   case GearReport::NONE:
  //   default:
  //     state_report().gear = 0;
  //     RCLCPP_WARN_THROTTLE(
  //       m_logger, m_clock, CLOCK_1_SEC, "Received invalid gear value from Dataspeed Ford DBW.");
  //     break;
  // }
  // m_seen_gear_rpt = true;
}

void DataspeedFordInterface::on_misc_report(const Misc1Report::SharedPtr & msg)
{
  // const float32_t speed_mps = msg->vehicle_speed * KPH_TO_MPS_RATIO * m_travel_direction;
  // const float32_t wheelbase = m_rear_axle_to_cog + m_front_axle_to_cog;
  // float32_t delta{0.0F};
  // float32_t beta{0.0F};
  // float32_t prev_speed_mps{0.0F};
  // float32_t dT{0.0F};

  // odometry().velocity_mps = speed_mps;

  // state_report().fuel = static_cast<uint8_t>(msg->fuel_level);

  // if (msg->drive_by_wire_enabled) {
  //   state_report().mode = VehicleStateReport::MODE_AUTONOMOUS;
  // } else {
  //   state_report().mode = VehicleStateReport::MODE_MANUAL;
  // }
  // m_dbw_state_machine->dbw_feedback(msg->by_wire_ready && !msg->general_driver_activity);

  // std::lock_guard<std::mutex> guard_vks(m_vehicle_kin_state_mutex);

  // /**
  //  * Input velocity is (assumed to be) measured at the rear axle, but we're
  //  * producing a velocity at the center of gravity.
  //  * Lateral velocity increases linearly from 0 at the rear axle to the maximum
  //  * at the front axle, where it is tan(δ)*v_lon.
  //  */
  // delta = m_vehicle_kin_state.state.front_wheel_angle_rad;
  // if (m_seen_misc_rpt && m_seen_wheel_spd_rpt) {
  //   prev_speed_mps = m_vehicle_kin_state.state.longitudinal_velocity_mps;
  // }
  // m_vehicle_kin_state.state.longitudinal_velocity_mps = speed_mps;
  // m_vehicle_kin_state.state.lateral_velocity_mps =
  //   (m_rear_axle_to_cog / wheelbase) * speed_mps * std::tan(delta);

  // m_vehicle_kin_state.header.frame_id = "odom";

  // // need >1 message in to calculate dT
  // if (!m_seen_misc_rpt) {
  //   m_seen_misc_rpt = true;
  //   m_vehicle_kin_state.header.stamp = msg->header.stamp;
  //   // Position = (0,0) at time = 0
  //   m_vehicle_kin_state.state.pose.position.x = 0.0;
  //   m_vehicle_kin_state.state.pose.position.y = 0.0;
  //   m_vehicle_kin_state.state.pose.orientation = motion::motion_common::from_angle(0.0);
  //   return;
  // }

  // // Calculate dT (seconds)
  // dT = static_cast<float32_t>(msg->header.stamp.sec - m_vehicle_kin_state.header.stamp.sec);
  // dT +=
  //   static_cast<float32_t>(msg->header.stamp.nanosec - m_vehicle_kin_state.header.stamp.nanosec)
  //   / 1000000000.0F;

  // if (dT < 0.0F) {
  //   RCLCPP_ERROR_THROTTLE(m_logger, m_clock, CLOCK_1_SEC, "Received inconsistent timestamps.");
  //   return;
  // }

  // m_vehicle_kin_state.header.stamp = msg->header.stamp;

  // if (m_seen_steering_rpt && m_seen_wheel_spd_rpt) {
  //   m_vehicle_kin_state.state.acceleration_mps2 = (speed_mps - prev_speed_mps) / dT;  // m/s^2

  //   beta = std::atan2(m_rear_axle_to_cog * std::tan(delta), wheelbase);
  //   m_vehicle_kin_state.state.heading_rate_rps = std::cos(beta) * std::tan(delta) / wheelbase;

  //   // Update position (x, y), yaw
  //   kinematic_bicycle_model(dT, &m_vehicle_kin_state);

  //   m_vehicle_kin_state_pub->publish(m_vehicle_kin_state);
  // }
}

void DataspeedFordInterface::on_steering_report(const SteeringReport::SharedPtr & msg)
{
  /* Steering -> tire angle conversion is linear except for extreme angles */
  const float32_t f_wheel_angle_rad =
    (msg->steering_wheel_angle * DEGREES_TO_RADIANS) / m_steer_to_tire_ratio;

  odometry().front_wheel_angle_rad = f_wheel_angle_rad;
  odometry().rear_wheel_angle_rad = 0.0F;

  std::lock_guard<std::mutex> guard_vks(m_vehicle_kin_state_mutex);
  m_vehicle_kin_state.state.front_wheel_angle_rad = f_wheel_angle_rad;
  m_vehicle_kin_state.state.rear_wheel_angle_rad = 0.0F;

  m_seen_steering_rpt = true;
  odometry().stamp = msg->header.stamp;
}

void DataspeedFordInterface::on_wheel_spd_report(const WheelSpeedReport::SharedPtr & msg)
{
  // Detect direction of travel
  float32_t fr = msg->front_right;
  float32_t fl = msg->front_left;
  float32_t rr = msg->rear_right;
  float32_t rl = msg->rear_left;

  if ((fr == 0.0F) && (fl == 0.0F) && (rr == 0.0F) && (rl == 0.0F)) {
    // car is not moving
    m_travel_direction = 0.0F;
  } else if ((fr >= 0.0F) && (fl >= 0.0F) && (rr >= 0.0F) && (rl >= 0.0F)) {
    // car is moving forward
    m_travel_direction = 1.0F;
  } else if ((fr <= 0.0F) && (fl <= 0.0F) && (rr <= 0.0F) && (rl <= 0.0F)) {
    // car is moving backward
    m_travel_direction = -1.0F;
  } else {
    // Wheels are moving different directions. This is probably bad.
    m_travel_direction = 0.0F;
    RCLCPP_WARN_THROTTLE(m_logger, m_clock, CLOCK_1_SEC, "Received inconsistent wheel speeds.");
  }

  m_seen_wheel_spd_rpt = true;
}

// Update x, y, heading, and heading_rate
void DataspeedFordInterface::kinematic_bicycle_model(float32_t dt, VehicleKinematicState * vks)
{
  const float32_t wheelbase = m_rear_axle_to_cog + m_front_axle_to_cog;

  // convert to yaw – copied from trajectory_spoofer.cpp
  // The below formula could probably be simplified if it would be derived directly for heading
  auto yaw = static_cast<float32_t>(motion::motion_common::to_angle(vks->state.pose.orientation));
  if (yaw < 0) {
    yaw += TAU;
  }

  /* δ: tire angle (relative to car's main axis)
   * φ: heading/yaw
   * β: direction of movement at point of reference (relative to car's main axis)
   * m_rear_axle_to_cog: distance of point of reference to rear axle
   * m_front_axle_to_cog: distance of point of reference to front axle
   * wheelbase: m_rear_axle_to_cog + m_front_axle_to_cog
   * x, y, v are at the point of reference
   * x' = v cos(φ + β)
   * y' = v sin(φ + β)
   * φ' = (cos(β)tan(δ)) / wheelbase
   * v' = a
   * β = arctan((m_rear_axle_to_cog*tan(δ))/wheelbase)
   */

  float32_t v0_lat = vks->state.lateral_velocity_mps;
  float32_t v0_lon = vks->state.longitudinal_velocity_mps;
  float32_t v0 = std::sqrt(v0_lat * v0_lat + v0_lon * v0_lon);
  float32_t delta = vks->state.front_wheel_angle_rad;
  float32_t a = vks->state.acceleration_mps2;
  float32_t beta = std::atan2(m_rear_axle_to_cog * std::tan(delta), wheelbase);

  // This is the direction in which the POI is moving at the beginning of the
  // integration step. "Course" may not be super accurate, but it's to
  // emphasize that the POI doesn't travel in the heading direction.
  const float32_t course = yaw + beta;

  // How much the yaw changes per meter traveled (at the reference point)
  const float32_t yaw_change = std::cos(beta) * std::tan(delta) / wheelbase;

  // How much the yaw rate
  const float32_t yaw_rate = yaw_change * v0;

  // Threshold chosen so as to not result in division by 0
  if (std::abs(yaw_rate) < 1e-18f) {
    vks->state.pose.position.x +=
      static_cast<float64_t>(std::cos(course) * (v0 * dt + 0.5f * a * dt * dt));
    vks->state.pose.position.y +=
      static_cast<float64_t>(std::sin(course) * (v0 * dt + 0.5f * a * dt * dt));
  } else {
    vks->state.pose.position.x += static_cast<float64_t>(
      (v0 + a * dt) / yaw_rate * std::sin(course + yaw_rate * dt) -
      v0 / yaw_rate * std::sin(course) +
      a / (yaw_rate * yaw_rate) * std::cos(course + yaw_rate * dt) -
      a / (yaw_rate * yaw_rate) * std::cos(course));
    vks->state.pose.position.y += static_cast<float64_t>(
      -(v0 + a * dt) / yaw_rate * std::cos(course + yaw_rate * dt) +
      v0 / yaw_rate * std::cos(course) +
      a / (yaw_rate * yaw_rate) * std::sin(course + yaw_rate * dt) -
      a / (yaw_rate * yaw_rate) * std::sin(course));
  }
  yaw += std::cos(beta) * std::tan(delta) / wheelbase * (v0 * dt + 0.5f * a * dt * dt);
  vks->state.pose.orientation = motion::motion_common::from_angle(yaw);

  // Rotations per second or rad per second?
  vks->state.heading_rate_rps = yaw_rate;
}

}  // namespace dataspeed_ford_interface
}  // namespace autoware
