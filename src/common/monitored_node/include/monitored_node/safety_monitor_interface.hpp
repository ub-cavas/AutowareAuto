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

#ifndef MONITORED_NODE__SAFETY_MONITOR_INTERFACE_HPP_
#define MONITORED_NODE__SAFETY_MONITOR_INTERFACE_HPP_


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wuseless-cast"
#include <rclcpp/rclcpp.hpp>
#pragma GCC diagnostic pop

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <string>
#include <tuple>
#include <memory>
#include <map>

#include "monitored_node/visibility_control.hpp"

using namespace std::chrono_literals; // NOLINT

namespace autoware
{
namespace common
{
namespace monitored_node
{

/// @brief The SafetyMonitorInterface is in charge of managing timers for monitoring and
///        communication with external monitors.
class MONITORED_NODE_PUBLIC SafetyMonitorInterface
{
public:
  using SharedPtr = std::shared_ptr<SafetyMonitorInterface>;
  static constexpr const char * SAFETY_MONITOR_INTERFACE_PREFIX = "SAFETY_MONITOR";
  static constexpr const char * DIAGNOSTIC_TOPIC = "/diagnostic";
  static constexpr const char * ARRIVED_TOO_SOON_EVENT_NAME = "arrived_too_soon";
  static constexpr const char * DID_NOT_RECEIVE_IN_TIME_EVENT_NAME = "did_not_receive_in_time";
  static constexpr const char * MIN_TIME_KEY_NAME = "min_time";
  static constexpr const char * MAX_TIME_KEY_NAME = "max_time";
  static constexpr const char * ELAPSED_TIME_KEY_NAME = "elapsed_time";
  static constexpr const char * TIMESTAMP_KEY_NAME = "timestamp";

  /// \brief Constructor
  /// \param[in] parent_node Pointer to the rclcpp::Node object which is being monitored
  /// \param[in] timer_callback_group Callback group to be used to invoke timer callbacks
  SafetyMonitorInterface(
    rclcpp::Node * parent_node,
    rclcpp::CallbackGroup::SharedPtr timer_callback_group)
  : m_parent_node(parent_node), m_timer_callback_group(timer_callback_group)
  {
    // Create a publisher to publish diagnostic messages. The topic name is hard coded so that all
    // nodes will publish to the same topic.
    m_diagnostic_publisher =
      m_parent_node->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
      DIAGNOSTIC_TOPIC, rclcpp::QoS(10));
  }

  /// \brief Expect an event to be raised between min_time and max_time. If the event is never
  ///        raised, an error event will be raised instead.
  /// \param[in] event_name Name of the event to look out for
  /// \param[in] min_time Minimum amount of time elapsed before the expected event arrives
  /// \param[in] max_time Maximum amount of time elapsed before the expected event arrives
  void expect_event(
    std::string event_name, std::chrono::milliseconds min_time,
    std::chrono::milliseconds max_time)
  {
    // sanity check the inputs
    if (min_time >= 0ms && max_time > min_time) {
      // check if a timer for this event already exists
      m_timers_mutex.lock();
      if (m_timers.find(event_name) == m_timers.end() && rclcpp::ok()) {
        // log information to console
        RCLCPP_INFO(
          m_parent_node->get_logger(),
          "expect event:%s, create timer with timeout: %lu ms, min_time %lu, %lu.",
          event_name.c_str(), max_time.count(), min_time.count(),
          m_parent_node->now().nanoseconds());

        // create a timer that expires at max_time
        auto wall_timer_ptr = m_parent_node->create_wall_timer(
          max_time, [this, event_name, min_time, max_time]() {
            // note time of timer callback
            auto event_timestamp = m_parent_node->now();

            // remove the timer
            this->m_timers_mutex.lock();
            if (m_timers.find(event_name) != m_timers.end()) {
              m_timers[event_name].wall_timer->cancel();
              m_timers.erase(event_name);
            } else {
              RCLCPP_ERROR(
                m_parent_node->get_logger(), "Did not find timer for event %s.",
                event_name);
            }
            this->m_timers_mutex.unlock();

            // publish an message to notify of a timeout event
            diagnostic_msgs::msg::DiagnosticStatus msg{};
            diagnostic_msgs::msg::KeyValue key_value_pair{};
            msg.level = msg.ERROR;
            msg.message = event_name + ":" + DID_NOT_RECEIVE_IN_TIME_EVENT_NAME;

            key_value_pair.key = TIMESTAMP_KEY_NAME;
            key_value_pair.value = std::to_string(event_timestamp.nanoseconds());
            msg.values.push_back(key_value_pair);

            key_value_pair.key = MAX_TIME_KEY_NAME;
            auto max_time_in_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(max_time).count();
            key_value_pair.value = std::to_string(max_time_in_ns);
            msg.values.push_back(key_value_pair);
            this->m_diagnostic_publisher->publish(msg);

            // log event to console
            RCLCPP_ERROR(
              m_parent_node->get_logger(),
              "did not receive event %s in time. max_interval: %lu ms.",
              event_name.c_str(), max_time.count());
          }, m_timer_callback_group);

        // save timer and other useful information in a class member
        m_timers[event_name] = {
          wall_timer_ptr,
          min_time,
          max_time,
          m_parent_node->now()
        };
      } else {
        RCLCPP_ERROR(
          m_parent_node->get_logger(), "already waiting for event %s, no new timer created.",
          event_name.c_str());
      }
      m_timers_mutex.unlock();
    } else {
      RCLCPP_ERROR(
        m_parent_node->get_logger(),
        "The min/max time (%llims, %llims) for event %s is not possible.",
        min_time.count(), max_time.count(), event_name.c_str());
    }
  }

  /// \brief Signal that an event has occurred.
  /// \param[in] event_name name of the event
  void emit_event(std::string event_name)
  {
    // note the time of the event
    auto event_timestamp = m_parent_node->now();

    // create and publish ros2 diagnostic message
    diagnostic_msgs::msg::DiagnosticStatus msg{};
    diagnostic_msgs::msg::KeyValue key_value_pair{};
    msg.level = msg.OK;
    msg.name = get_msg_name();
    msg.message = event_name;
    msg.hardware_id = "";
    key_value_pair.key = TIMESTAMP_KEY_NAME;
    key_value_pair.value = std::to_string(event_timestamp.nanoseconds());
    msg.values.push_back(key_value_pair);
    m_diagnostic_publisher->publish(msg);

    // Print to concole log
    RCLCPP_INFO(
      m_parent_node->get_logger(), "Emit event %s:%lu",
      event_name.c_str(), event_timestamp.nanoseconds());

    // If a timer for this event exist and is not expired, we should cancel the timer and check the
    // min_time criteria for this event.
    this->m_timers_mutex.lock();
    if (m_timers.find(event_name) != m_timers.end() &&
      !m_timers[event_name].wall_timer->is_ready())
    {
      // cancel timer
      m_timers[event_name].wall_timer->cancel();
      auto min_time = m_timers[event_name].min_time;
      auto elapsed_time = event_timestamp - m_timers[event_name].timestamp;
      m_timers.erase(event_name);

      // check if the event actually arrived too early
      if (elapsed_time < min_time) {
        // publish an error message
        msg.level = msg.ERROR;
        msg.message = event_name + ":" + ARRIVED_TOO_SOON_EVENT_NAME;
        key_value_pair.key = MIN_TIME_KEY_NAME;
        auto min_time_in_ns =
          std::chrono::duration_cast<std::chrono::nanoseconds>(min_time).count();
        key_value_pair.value = std::to_string(min_time_in_ns);
        msg.values.push_back(key_value_pair);
        key_value_pair.key = ELAPSED_TIME_KEY_NAME;
        auto elapsed_time_in_ns = elapsed_time.nanoseconds();
        key_value_pair.value = std::to_string(elapsed_time_in_ns);
        m_diagnostic_publisher->publish(msg);

        // log error to console
        RCLCPP_ERROR(
          m_parent_node->get_logger(),
          "event %s arrived at %lu. arrived_too_early: min_time %lu ms, elapsed_time:%lu ms.",
          event_name.c_str(),
          event_timestamp.nanoseconds() * 1e-6, min_time.count(), elapsed_time_in_ns * 1e-6);
      }
    }
    this->m_timers_mutex.unlock();
  }

private:
  /// \brief Structure containing timer-related information to be stored in other class members.
  struct TimerInfo
  {
    rclcpp::TimerBase::SharedPtr wall_timer;
    std::chrono::milliseconds min_time;
    std::chrono::milliseconds max_time;
    rclcpp::Time timestamp;
  };

  // publisher to publish diagnostic message
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr m_diagnostic_publisher{};
  // pointer to the ros node under monitoring
  rclcpp::Node * m_parent_node{};
  // structure to keep track of all the started timers indexed by the event name
  std::map<std::string, TimerInfo> m_timers{};
  // Mutex to control multi threaded access to the timers
  std::mutex m_timers_mutex{};
  // Shared pointer to the callback group used to invoke timer callbacks.
  rclcpp::CallbackGroup::SharedPtr m_timer_callback_group{};

  /// \brief get the diagnostic message's "name" field. This is a combination of a hard-coded
  ///        prefix and the parent node's name
  /// \return disagnostic message name as a std::string
  inline std::string get_msg_name()
  {
    return std::string(SAFETY_MONITOR_INTERFACE_PREFIX) + ":" + m_parent_node->get_name();
  }
};

}  // namespace monitored_node
}  // namespace common
}  // namespace autoware

#endif  // MONITORED_NODE__SAFETY_MONITOR_INTERFACE_HPP_
