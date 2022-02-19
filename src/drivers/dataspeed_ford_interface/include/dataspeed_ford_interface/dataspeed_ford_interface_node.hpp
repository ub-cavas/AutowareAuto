#ifndef DATASPEED_FORD_INTERFACE__DATASPEED_FORD_INTERFACE_NODE_HPP_
#define DATASPEED_FORD_INTERFACE__DATASPEED_FORD_INTERFACE_NODE_HPP_

#include <dataspeed_ford_interface/visibility_control.hpp>

#include <vehicle_interface/vehicle_interface_node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <string>

namespace autoware
{
namespace dataspeed_ford_interface
{

/// \class DataspeedFordInterfaceNode
/// \brief ROS 2 Node for hello world.
class DATASPEED_FORD_INTERFACE_PUBLIC DataspeedFordInterfaceNode
  : public ::autoware::drivers::vehicle_interface::VehicleInterfaceNode
{
public:
  /// \brief default constructor, starts driver
  /// \param[in] options Options for the node
  /// \throw runtime error if failed to start threads or configure driver
  explicit DataspeedFordInterfaceNode(const rclcpp::NodeOptions & options);

  rcl_interfaces::msg::SetParametersResult on_parameter_set(
    const std::vector<rclcpp::Parameter> & parameters);

private:
  OnSetParametersCallbackHandle::SharedPtr m_param_callback_handle;
};
}  // namespace dataspeed_ford_interface
}  // namespace autoware

#endif  // DATASPEED_FORD_INTERFACE__DATASPEED_FORD_INTERFACE_NODE_HPP_
