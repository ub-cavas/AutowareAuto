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

#ifndef MONITORED_NODE__MONITORED_PUBLISHER_HPP_
#define MONITORED_NODE__MONITORED_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>

#include <chrono>
#include <memory>
#include <string>

#include "monitored_node/visibility_control.hpp"

using namespace std::chrono_literals; // NOLINT

namespace autoware
{
namespace common
{
namespace monitored_node
{

/// \brief Wrapper of a rclcpp::publisher class to implement monitoring functionalities
/// \tparam MessageT Message type.
template<typename MessageT>
class MONITORED_NODE_PUBLIC MonitoredPublisher
{
public:
  using SharedPtr = std::shared_ptr<MonitoredPublisher<MessageT>>;
  using IntervalTopicType = std_msgs::msg::UInt64MultiArray;

  /// \brief Constructor
  /// \param[in] topic_name The topic for this publisher to publish on.
  /// \param[in] qos The Quality of Service settings for the publisher.
  /// \param[in] min_publishing_interval The minimum publishing interval for this publisher in ms.
  /// \param[in] max_publishing_interval The maximum publishing interval for this publisher in ms.
  /// \param[in] parent_node Pointer to the node that this publisher belong to.
  MonitoredPublisher(
    const std::string & topic_name, const rclcpp::QoS & qos,
    std::chrono::milliseconds min_publishing_interval,
    std::chrono::milliseconds max_publishing_interval,
    rclcpp::Node * parent_node)
  {
    // create the publisher on the main user suppied topic
    m_main_publisher = parent_node->create_publisher<MessageT>(topic_name, qos);

    // initialise the interval publishers
    init_interval_publishers(
      min_publishing_interval, max_publishing_interval, parent_node,
      topic_name);
  }

  /// \brief Constructor
  /// \param[in] topic_name The topic for this publisher to publish on.
  /// \param[in] qos The Quality of Service settings for the publisher.
  /// \param[in] min_publishing_interval_future The future object for the minimum publishing
  ///                                           interval. Usually obtained from a
  ///                                           MonitoredSubscription object.
  /// \param[in] max_publishing_interval_future The future object for the maximum publishing
  ///                                           interval. Usually obtained from a
  ///                                           MonitoredSubscription object.
  /// \param[in] parent_node Pointer to the node that this publisher belong to.
  MonitoredPublisher(
    const std::string & topic_name, const rclcpp::QoS & qos,
    std::shared_future<std::chrono::milliseconds> min_publishing_interval_future,
    std::shared_future<std::chrono::milliseconds> max_publishing_interval_future,
    rclcpp::Node * parent_node)
  {
    // create the publisher on the main user supplied topic
    m_main_publisher = parent_node->create_publisher<MessageT>(topic_name, qos);

    // create timer to periodically check the readiness of the futures
    m_wall_timer = parent_node->create_wall_timer(
      100ms,
      [this, topic_name, parent_node, min_publishing_interval_future,
      max_publishing_interval_future]() {
        if (max_publishing_interval_future.wait_for(0s) == std::future_status::ready &&
        min_publishing_interval_future.wait_for(0s) == std::future_status::ready)
        {
          // clean up the timer
          this->m_wall_timer->cancel();
          this->m_wall_timer = nullptr;

          // initialise interval publishers
          this->init_interval_publishers(
            std::chrono::milliseconds(min_publishing_interval_future.get()),
            std::chrono::milliseconds(max_publishing_interval_future.get()),
            parent_node, topic_name);
        }
      });
  }

  /// \brief Getter function for the rclcpp:Publisher object
  /// \return The rclcpp:Publisher object for the main topic publisher
  auto get_pub_shared_ptr() {return m_main_publisher;}

private:
  // Shared pointer to rclcpp:Publisher object for the main publishing topic
  typename rclcpp::Publisher<MessageT>::SharedPtr m_main_publisher{};

  // Shared pointer to rclcpp:Publisher object for the min/max interval topic
  rclcpp::Publisher<IntervalTopicType>::SharedPtr m_interval_publisher{};

  // Shared pointer to a timer used to resolve publishing interval futures.
  rclcpp::TimerBase::SharedPtr m_wall_timer{};

  /// \brief Initialise the min/max interval publisher and publish the min/max interval messages
  /// \param[in] min_publishing_interval The minimum publishing interval for this publisher in ms.
  /// \param[in] max_publishing_interval The maximum publishing interval for this publisher in ms.
  /// \param[in] parent_node Pointer to the node that this publisher belong to.
  /// \param[in] topic_name The topic for this publisher to publish on.
  void init_interval_publishers(
    std::chrono::milliseconds min_publishing_interval,
    std::chrono::milliseconds max_publishing_interval,
    rclcpp::Node * parent_node,
    std::string topic_name)
  {
    if (max_publishing_interval > 0ms && min_publishing_interval >= 0ms) {
      // create the interval publisher
      m_interval_publisher = parent_node->create_publisher<IntervalTopicType>(
        get_publish_interval_topic_name(topic_name), get_latched_qos_profile());

      // send interval message
      IntervalTopicType msg;
      msg.data.push_back(min_publishing_interval.count());
      msg.data.push_back(max_publishing_interval.count());
      m_interval_publisher->publish(msg);
    }
  }

  /// \brief Get the topic name of the publishing interval topic
  /// \param[in] topic_name The name of the main publishing topic
  /// \return String topic name of the publishing interval topic for this publisher
  static auto get_publish_interval_topic_name(const std::string & topic_name)
  {
    return topic_name + "/min_max_publish_interval";
  }

  /// \brief Get the QoS profile used for the interval publishers
  /// \return A latched QoS profile
  static auto get_latched_qos_profile()
  {
    return rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  }

  template<typename T>
  friend class MonitoredSubscription;
};

}  // namespace monitored_node
}  // namespace common
}  // namespace autoware

#endif  // MONITORED_NODE__MONITORED_PUBLISHER_HPP_
