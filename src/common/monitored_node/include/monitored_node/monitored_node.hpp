// Copyright 2021 Arm Ltd.
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

#ifndef MONITORED_NODE__MONITORED_NODE_HPP_
#define MONITORED_NODE__MONITORED_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <utility>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "monitored_node/safety_monitor_interface.hpp"
#include "monitored_node/monitored_publisher.hpp"
#include "monitored_node/monitored_subscription.hpp"
#include "monitored_node/visibility_control.hpp"

using namespace std::chrono_literals; // NOLINT

namespace autoware
{
namespace common
{
namespace monitored_node
{

/// \brief Main interface to create monitored node. MonitoredNode inherits from rclcpp::Node and
///        provide its own monitored publisher and subscription creation functions.
class MONITORED_NODE_PUBLIC MonitoredNode : public rclcpp::Node
{
public:
  using SharedPtr = std::shared_ptr<MonitoredNode>;

  /// \brief Constructor
  /// \param[in] node_name Node name for this node.
  /// \param[in] node_options Node options for this node.
  MonitoredNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node(node_name, node_options)
  {
    // Create callback groups so that timer and subscription callbacks can be invoked in separate
    // threads. This only applies when a multi-thread executor is used.
    m_timer_callback_group = create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    m_subscription_callback_group = create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

    // Initialise safety monitor interface
    m_safety_monitor_interface = std::make_shared<SafetyMonitorInterface>(
      this,
      m_timer_callback_group);
  }

  /// \brief Create a monitored subscription. Callback frequency and callback duration will be
  ///        monitored. This interface mirrors that of the rclcpp:Node API except that it asks for
  ///        an extra parameter for max callback duration.
  /// \tparam MessageT Message type.
  /// \param[in] topic_name topic name to subscribe to
  /// \param[in] qos quality of service
  /// \param[in] callback user defined callback function invoked when a message is received
  /// \param[in] max_callback_duration maximum callback duration
  /// \return shared pointer to a MonitoredSubscription object
  template<typename MessageT>
  typename MonitoredSubscription<MessageT>::SharedPtr
  create_monitored_subscription(
    const std::string & topic_name, const rclcpp::QoS & qos,
    std::function<void(const std::shared_ptr<MessageT>)> && callback,
    std::chrono::milliseconds max_callback_duration = 0ms)
  {
    auto max_callback_duration_parameter_value = declare_parameter(
      topic_name + "." + m_max_callback_duration_param_name, 0);

    // if a parameter was specificed, overwrite the compile time value of
    // max_callback_duration
    if (max_callback_duration_parameter_value != 0) {
      max_callback_duration = std::chrono::milliseconds(max_callback_duration_parameter_value);
    }

    // create a monitored subscription object
    return std::make_shared<MonitoredSubscription<MessageT>>(
      topic_name, qos, std::move(callback), max_callback_duration,
      m_safety_monitor_interface, this, m_subscription_callback_group);
  }

  /// \brief Create a monitored publisher. The min and max publishing intervals are propagated to
  ///        the subscriber for monitoring. This interface mirrors that of the rclcpp:Node API
  ///        except that it asks for extra parameter for min/max publishing interval.
  /// \details The min/max_publish_interval parameters can be of std::chrono::milliseconds type or a
  ///          std::shared_future object obtained obtained from a MonitoredSubscription using the
  ///          get_min/max_interval_future() API. The 2 parameters have to be of the same type.
  /// \tparam MessageT Message type.
  /// \tparam PublishIntervalT Type of the publishing intervals. They can either be
  ///         std::chrono::milliseconds or std::shared_future<std::chrono::milliseconds>.
  /// \param[in] topic_name topic name to subscribe to
  /// \param[in] qos quality of service
  /// \param[in] min_publish_interval minimum interval between 2 messages in ms. This parameter can
  ///                                 be of std::chrono::milliseconds type or a std::shared_future
  ///                                 object which is obtained obtained from a MonitoredSubscription
  ///                                 object using the get_min_interval_future() API.
  /// \param[in] max_publish_interval maximum interval between 2 messages in ms. This parameter can
  ///                                 be of std::chrono::milliseconds type or a std::shared_future
  ///                                 object which is obtained obtained from a MonitoredSubscription
  ///                                 object using the get_max_interval_future() API.
  /// \return shared pointer to a rclcpp::Publisher object. This is the same as the rclcpp API so
  ///         that any code minipulating the publisher object does not need to change.
  template<typename MessageT, typename PublishIntervalT>
  auto create_monitored_publisher(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    PublishIntervalT min_publish_interval = 0ms,
    PublishIntervalT max_publish_interval = 0ms)
  {
    // declare parameters for the max and min publishing interval
    auto max_publish_interval_parameter_value =
      declare_parameter(topic_name + "." + m_max_publish_interval_param_name, 0);
    auto min_publish_interval_parameter_value =
      declare_parameter(topic_name + "." + m_min_publish_interval_param_name, 0);

    // parameters always overwrite hard coded values
    if (min_publish_interval_parameter_value != 0 &&
      max_publish_interval_parameter_value != 0)
    {
      auto overwrite_min_publish_interval =
        std::chrono::milliseconds(min_publish_interval_parameter_value);
      auto overwrite_max_publish_interval =
        std::chrono::milliseconds(max_publish_interval_parameter_value);

      return std::make_shared<MonitoredPublisher<MessageT>>(
        topic_name, qos, overwrite_min_publish_interval,
        overwrite_max_publish_interval, this);
    }

    return std::make_shared<MonitoredPublisher<MessageT>>(
      topic_name, qos, min_publish_interval,
      max_publish_interval, this);
  }

private:
  // store a shared pointer to the safety monitor interface
  std::shared_ptr<SafetyMonitorInterface> m_safety_monitor_interface{};

  // hard coded parameter names for a monitored node
  static constexpr const char * m_min_publish_interval_param_name{"min_publish_interval_ms"};
  static constexpr const char * m_max_publish_interval_param_name{"max_publish_interval_ms"};
  static constexpr const char * m_max_callback_duration_param_name{"max_callback_duration_ms"};

  // shared pointers to callback groups to be used with timers and subscriptions
  rclcpp::callback_group::CallbackGroup::SharedPtr m_timer_callback_group{};
  rclcpp::callback_group::CallbackGroup::SharedPtr m_subscription_callback_group{};
};

}  // namespace monitored_node
}  // namespace common
}  // namespace autoware

#endif  // MONITORED_NODE__MONITORED_NODE_HPP_
