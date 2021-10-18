// Copyright 2019 Christopher Ho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mpc_controller_nodes/mpc_controller_node.hpp"

#include <memory>
#include <string>
#include <utility>

namespace motion
{
namespace control
{
namespace mpc_controller_nodes
{
////////////////////////////////////////////////////////////////////////////////
MpcControllerNode::MpcControllerNode(const std::string & name, const std::string & ns)
: ControllerBaseNode{name, ns}
{
  using mpc_controller::Real;
  using mpc_controller::LimitsConfig;
  const LimitsConfig limits{
    {
      static_cast<Real>(
        declare_parameter<double>("controller.limits.min_longitudinal_velocity_mps")),
      static_cast<Real>(
        declare_parameter<double>("controller.limits.max_longitudinal_velocity_mps"))
    },
    {
      static_cast<Real>(
        declare_parameter<double>("controller.limits.min_lateral_velocity_mps")),
      static_cast<Real>(
        declare_parameter<double>("controller.limits.max_lateral_velocity_mps"))
    },
    {
      static_cast<Real>(declare_parameter<double>("controller.limits.min_acceleration_mps2")),
      static_cast<Real>(declare_parameter<double>("controller.limits.max_acceleration_mps2"))
    },
    {
      static_cast<Real>(declare_parameter<double>("controller.limits.min_yaw_rate_rps")),
      static_cast<Real>(declare_parameter<double>("controller.limits.max_yaw_rate_rps"))
    },
    {
      static_cast<Real>(declare_parameter<double>("controller.limits.min_jerk_mps3")),
      static_cast<Real>(declare_parameter<double>("controller.limits.max_jerk_mps3"))
    },
    {
      static_cast<Real>(declare_parameter<double>("controller.limits.min_steer_angle_rad")),
      static_cast<Real>(declare_parameter<double>("controller.limits.max_steer_angle_rad"))
    },
    {
      static_cast<Real>(
        declare_parameter<double>("controller.limits.min_steer_angle_rate_rps")),
      static_cast<Real>(
        declare_parameter<double>("controller.limits.max_steer_angle_rate_rps"))
    },
  };
  using mpc_controller::VehicleConfig;
  const VehicleConfig vehicle_param{
    static_cast<Real>(declare_parameter<double>("vehicle.cg_to_front_m")),
    static_cast<Real>(declare_parameter<double>("vehicle.cg_to_rear_m")),
    static_cast<Real>(declare_parameter<double>("vehicle.front_corner_stiffness")),
    static_cast<Real>(declare_parameter<double>("vehicle.rear_corner_stiffness")),
    static_cast<Real>(declare_parameter<double>("vehicle.mass_kg")),
    static_cast<Real>(declare_parameter<double>("vehicle.yaw_inertia_kgm2")),
    static_cast<Real>(declare_parameter<double>("vehicle.width_m")),
    static_cast<Real>(declare_parameter<double>("vehicle.front_overhang_m")),
    static_cast<Real>(declare_parameter<double>("vehicle.rear_overhang_m"))
  };
  using mpc_controller::BehaviorConfig;
  using controller_common::ControlReference;
  auto ref_type = ControlReference::SPATIAL;
  if (declare_parameter<bool>("controller.behavior.is_temporal_reference")) {
    ref_type = ControlReference::TEMPORAL;
  }
  const BehaviorConfig behavior{
    static_cast<Real>(declare_parameter<double>("controller.behavior.stop_rate_mps2")),
    std::chrono::milliseconds(declare_parameter<int64_t>("controller.behavior.time_step_ms")),
    ref_type
  };

  using mpc_controller::OptimizationConfig;
  using mpc_controller::StateWeight;
  const OptimizationConfig weights{
    StateWeight{
      static_cast<Real>(declare_parameter<double>("controller.weights.nominal.pose")),
      static_cast<Real>(declare_parameter<double>("controller.weights.nominal.heading")),
      static_cast<Real>(
        declare_parameter<double>("controller.weights.nominal.longitudinal_velocity")),
      static_cast<Real>(
        declare_parameter<double>("controller.weights.nominal.lateral_velocity")),
      static_cast<Real>(declare_parameter<double>("controller.weights.nominal.yaw_rate")),
      static_cast<Real>(declare_parameter<double>("controller.weights.nominal.acceleration")),
      static_cast<Real>(declare_parameter<double>("controller.weights.nominal.jerk")),
      static_cast<Real>(declare_parameter<double>("controller.weights.nominal.steer_angle")),
      static_cast<Real>(
        declare_parameter<double>("controller.weights.nominal.steer_angle_rate")),
    },
    StateWeight{
      static_cast<Real>(declare_parameter<double>("controller.weights.terminal.pose")),
      static_cast<Real>(declare_parameter<double>("controller.weights.terminal.heading")),
      static_cast<Real>(
        declare_parameter<double>("controller.weights.terminal.longitudinal_velocity")),
      static_cast<Real>(
        declare_parameter<double>("controller.weights.terminal.lateral_velocity")),
      static_cast<Real>(declare_parameter<double>("controller.weights.terminal.yaw_rate")),
      static_cast<Real>(
        declare_parameter<double>("controller.weights.terminal.acceleration")),
      static_cast<Real>(declare_parameter<double>("controller.weights.terminal.jerk")),
      static_cast<Real>(declare_parameter<double>("controller.weights.terminal.steer_angle")),
      static_cast<Real>(
        declare_parameter<double>("controller.weights.terminal.steer_angle_rate")),
    }
  };

  using mpc_controller::Interpolation;
  auto interpolation = Interpolation::NO;
  if (declare_parameter<bool>("controller.interpolation")) {
    interpolation = Interpolation::YES;
  }
  const auto sample_tolerance_ms =
    std::chrono::milliseconds(declare_parameter<int64_t>("controller.sample_tolerance_ms"));

  const auto control_lookahead_ms =
    std::chrono::milliseconds(declare_parameter<int64_t>("controller.control_lookahead_ms"));

  auto controller = std::make_unique<mpc_controller::MpcController>(
    mpc_controller::Config{
          limits,
          vehicle_param,
          behavior,
          weights,
          sample_tolerance_ms,
          control_lookahead_ms,
          interpolation});
  // I argue this is ok for the following reasons:
  // The parent class, ControllerBaseNode, has unique ownership of the controller, and the timer
  // only has a non-owning pointer. This is fine because the timer can never go out of scope before
  // the base class (and thus the owning pointer)
  const auto ctrl_ptr = controller.get();
  set_controller(std::move(controller));

  const auto cycle_duration =
    std::chrono::milliseconds{declare_parameter<int64_t>("debug_trajectory_publish_period_ms")};
  if (decltype(cycle_duration)::zero() != cycle_duration) {
    m_debug_traj_pub = create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
      "mpc_debug_computed_trajectory",
      rclcpp::QoS{10LL});
    const auto debug_publish = [this, ctrl_ptr]() -> void {
        auto traj = ctrl_ptr->get_computed_trajectory();
        traj.header.frame_id = "map";
        m_debug_traj_pub->publish(traj);
      };
    m_debug_timer = create_wall_timer(cycle_duration, debug_publish);
  }
}

////////////////////////////////////////////////////////////////////////////////
MpcControllerNode::MpcControllerNode(
  const std::string & name,
  const std::string & ns,
  const std::string & command_topic,
  const std::string & state_topic,
  const std::string & tf_topic,
  const std::string & trajectory_topic,
  const std::string & diagnostic_topic,
  const std::string & static_tf_topic,
  const mpc_controller::Config & config)
: ControllerBaseNode{
    name,
    ns,
    command_topic,
    state_topic,
    tf_topic,
    trajectory_topic,
    diagnostic_topic,
    static_tf_topic}
{
  set_controller(std::make_unique<mpc_controller::MpcController>(config));
}
}  // namespace mpc_controller_nodes
}  // namespace control
}  // namespace motion
