#include "dataspeed_ford_interface/dataspeed_ford_interface_node.hpp"
#include "dataspeed_ford_interface/dataspeed_ford_interface.hpp"

#include <common/types.hpp>

#include <memory>
#include <string>
#include <unordered_set>

namespace autoware
{
namespace dataspeed_ford_interface
{
using autoware::common::types::float32_t;
using autoware::drivers::vehicle_interface::ViFeature;

rcl_interfaces::msg::SetParametersResult DataspeedFordInterfaceNode::on_parameter_set(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result{};

  DataspeedFordInterface * interface =
    dynamic_cast<DataspeedFordInterface *>(this->get_interface().get());

  if (!interface) {
    result.successful = false;
    result.reason = "Cannot cast type to DataspeedFordInterface *";
    return result;
  }

  // update class attributes
  for (const auto & param : parameters) {
    if (param.get_name() == "dataspeed_ford.acceleration_control.kp") {
      // set_kp()
      interface->set_acceleration_control_kp(static_cast<float32_t>(param.as_double()));
    } else if (param.get_name() == "dataspeed_ford.acceleration_control.ki") {
      // set_ki()
      interface->set_acceleration_control_ki(static_cast<float32_t>(param.as_double()));
    } else if (param.get_name() == "dataspeed_ford.acceleration_control.kd") {
      // set_kd()
      interface->set_acceleration_control_kd(static_cast<float32_t>(param.as_double()));
    }
  }

  result.successful = true;
  result.reason = "success";
  return result;
}

DataspeedFordInterfaceNode::DataspeedFordInterfaceNode(const rclcpp::NodeOptions & options)
: VehicleInterfaceNode {
    "dataspeed_ford_interface",
    std::unordered_set<ViFeature> {
        ViFeature::HEADLIGHTS,
        ViFeature::HORN,
        ViFeature::WIPERS,
        ViFeature::GEAR,
      },
      options
}
{
  // initial values 
  rclcpp::ParameterValue pid_initial_value{0.0};

  // initiate sliders for P gain -- throttle controller
  rcl_interfaces::msg::ParameterDescriptor p_descriptor;
  rcl_interfaces::msg::FloatingPointRange p_range;
  p_range.from_value = 0;
  p_range.to_value = 10;
  p_range.step = 0.01;
  p_descriptor.floating_point_range.push_back(p_range);

  // initiate sliders for P gain -- throttle controller
  rcl_interfaces::msg::ParameterDescriptor i_descriptor;
  rcl_interfaces::msg::FloatingPointRange i_range;
  i_range.from_value = 0;
  i_range.to_value = 10;
  i_range.step = 0.01;
  i_descriptor.floating_point_range.push_back(i_range);

  // initiate sliders for P gain -- throttle controller
  rcl_interfaces::msg::ParameterDescriptor d_descriptor;
  rcl_interfaces::msg::FloatingPointRange d_range;
  d_range.from_value = 0;
  d_range.to_value = 10;
  d_range.step = 0.01;
  d_descriptor.floating_point_range.push_back(d_range);

  auto interface = std::make_unique<DataspeedFordInterface>(
    *this,
    declare_parameter("dataspeed_ford.ecu_build_num").get<uint16_t>(),
    declare_parameter("dataspeed_ford.front_axle_to_cog").get<float32_t>(),
    declare_parameter("dataspeed_ford.rear_axle_to_cog").get<float32_t>(),
    declare_parameter("dataspeed_ford.steer_to_tire_ratio").get<float32_t>(),
    declare_parameter("dataspeed_ford.max_steer_angle").get<float32_t>(),
    get_state_machine().get_config().accel_limits().max(),
    get_state_machine().get_config().accel_limits().min(),
    declare_parameter("dataspeed_ford.acceleration_positive_jerk_limit").get<float32_t>(),
    declare_parameter("dataspeed_ford.deceleration_negative_jerk_limit").get<float32_t>(),
    declare_parameter("dataspeed_ford.pub_period").get<uint32_t>(),
    declare_parameter("dataspeed_ford.acceleration_control.kp", pid_initial_value, p_descriptor)
      .get<float32_t>(),
    declare_parameter("dataspeed_ford.acceleration_control.ki", pid_initial_value, i_descriptor)
      .get<float32_t>(),
    declare_parameter("dataspeed_ford.acceleration_control.kd", pid_initial_value, d_descriptor)
      .get<float32_t>(),
    declare_parameter("dataspeed_ford.acceleration_control.deadzone_min").get<float32_t>(),
    declare_parameter("dataspeed_ford.acceleration_control.deadzone_max").get<float32_t>());
  set_interface(std::move(interface));

  m_param_callback_handle = this->add_on_set_parameters_callback(
    std::bind(&DataspeedFordInterfaceNode::on_parameter_set, this, std::placeholders::_1));
}
}  // namespace dataspeed_ford_interface
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::dataspeed_ford_interface::DataspeedFordInterfaceNode)
