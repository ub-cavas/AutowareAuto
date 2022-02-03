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

#ifndef MONITORED_NODE__MONITORED_SUBSCRIPTION_HPP_
#define MONITORED_NODE__MONITORED_SUBSCRIPTION_HPP_


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wuseless-cast"
#include <rclcpp/rclcpp.hpp>
#pragma GCC diagnostic pop

#include <std_msgs/msg/u_int32.hpp>

#include <chrono>
#include <ctime>
#include <memory>
#include <string>

#include "monitored_node/safety_monitor_interface.hpp"
#include "monitored_node/monitored_publisher.hpp"
#include "monitored_node/visibility_control.hpp"

using namespace std::chrono_literals; // NOLINT

namespace autoware
{
namespace common
{
namespace monitored_node
{

/// \brief Wrapper of a rclcpp::subscription class to implement monitoring functionalities
/// \tparam MessageT Message type.
template<typename MessageT>
class MONITORED_NODE_PUBLIC MonitoredSubscription
{
public:
  using SharedPtr = std::shared_ptr<MonitoredSubscription<MessageT>>;
  using CallbackParamT = const std::shared_ptr<MessageT>;
  using CallbackT = std::function<void (CallbackParamT)>;
  using IntervalTopicType = MonitoredPublisher<int>::IntervalTopicType;

  /// \brief Constructor
  /// \param[in] topic_name Name of the topic to subscribe to.
  /// \param[in] qos QoS profile for Subscription.
  /// \param[in] callback User defined callback to call when a message is received.
  /// \param[in] max_callback_duration The maximum amount of time the callback can take.
  /// \param[in] safety_monitor_interface Interface use to monitor and emit events.
  /// \param[in] parent_node Pointer to the node that this subscription belong to.
  /// \param[in] subscription_callback_group Group used to invoke user defined subscription
  ///                                        callback.
  MonitoredSubscription(
    const std::string & topic_name, const rclcpp::QoS & qos,
    CallbackT && callback, std::chrono::milliseconds max_callback_duration,
    SafetyMonitorInterface::SharedPtr safety_monitor_interface,
    rclcpp::Node * parent_node,
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group)
  : m_topic_name(topic_name), m_callback(callback),
    m_safety_monitor_interface(safety_monitor_interface),
    m_max_callback_duration(max_callback_duration)
  {
    // define a subscription option with the specified callback group
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = subscription_callback_group;

    // create the subscription using the normal rclcpp interface but passing in callback_wrapper
    // instead of the user callback
    m_sub = parent_node->create_subscription<MessageT>(
      topic_name, qos,
      std::bind(
        &MonitoredSubscription::callback_wrapper, this,
        std::placeholders::_1), subscription_options);

    // obtain publishing interval information from the publisher to the topic
    m_min_interval_future = m_min_interval_promise.get_future();
    m_max_interval_future = m_max_interval_promise.get_future();

    m_publish_interval_sub = parent_node->create_subscription<IntervalTopicType>(
      MonitoredPublisher<int>::get_publish_interval_topic_name(topic_name),
      MonitoredPublisher<int>::get_latched_qos_profile(),
      [this, topic_name](IntervalTopicType::SharedPtr msg) {
        try {
          this->m_min_interval_promise.set_value(std::chrono::milliseconds(msg->data[0]));
          this->m_max_interval_promise.set_value(std::chrono::milliseconds(msg->data[1]));
        } catch (const std::future_error & e) {
          // there is no guarantee of exactly once delivery, the value maybe already set.
          if (e.code() != std::future_errc::promise_already_satisfied) {
            throw e;
          }
        }
      }, subscription_options);
  }

  /// \brief Getter for the shared pointer of the main subscription object
  auto get_sub_shared_ptr() {return m_sub;}

  /// \brief Getter for the min interval future. This can be used to create further monitored
  ///        publisher objects that inherits the interval properties.
  auto get_min_interval_future() {return m_min_interval_future;}

  /// \brief Getter for the max interval future. This can be used to create further monitored
  ///        publisher objects that inherits the interval properties.
  auto get_max_interval_future() {return m_max_interval_future;}

private:
  // User define callback. Called from the callback wrapper.
  CallbackT m_callback{};
  // Shared pointer of the rclcpp subscription object pointint to the main user defined topic
  typename rclcpp::Subscription<MessageT>::SharedPtr m_sub{};
  // Shared pointer of the rclcpp subscription object pointing to the interval topic
  rclcpp::Subscription<IntervalTopicType>::SharedPtr m_publish_interval_sub{};
  // Name of the topic being subscribed.
  std::string m_topic_name{};
  // Promise and futures to set/get the interval properties of the topic being subscribed.
  std::promise<std::chrono::milliseconds> m_min_interval_promise{};
  std::promise<std::chrono::milliseconds> m_max_interval_promise{};
  std::shared_future<std::chrono::milliseconds> m_min_interval_future{};
  std::shared_future<std::chrono::milliseconds> m_max_interval_future{};
  // The maximum allowable time taken in the user callback
  std::chrono::milliseconds m_max_callback_duration{};
  // Shared pointer to the safety monitor interface
  SafetyMonitorInterface::SharedPtr m_safety_monitor_interface{};

  /// \brief Wraps the user defined callback in order to insert monitoring function calls around it.
  void callback_wrapper(const typename MessageT::SharedPtr msg) const
  {
    // test if we received min max interval values from the publisher yet.
    auto monitor_callback =
      this->m_min_interval_future.wait_for(0s) == std::future_status::ready &&
      this->m_max_interval_future.wait_for(0s) == std::future_status::ready;

    if (monitor_callback) {
      // publish callback start event
      m_safety_monitor_interface->emit_event(m_topic_name + ":callback_start");

      // expect the next callback_start event to occur within interval time limits
      m_safety_monitor_interface->expect_event(
        m_topic_name + ":callback_start",
        m_min_interval_future.get(),
        m_max_interval_future.get());

      // expect callback end within m_max_callback_duration.
      m_safety_monitor_interface->expect_event(
        m_topic_name + ":callback_end",
        0ms, m_max_callback_duration);
    }

    // invoke user callback
    m_callback(msg);

    // publish callback end event
    if (monitor_callback) {
      m_safety_monitor_interface->emit_event(m_topic_name + ":callback_end");
    }
  }
};

}  // namespace monitored_node
}  // namespace common
}  // namespace autoware

#endif  // MONITORED_NODE__MONITORED_SUBSCRIPTION_HPP_
