// Copyright 2021 Arm Limited and Contributors.
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

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wuseless-cast"
#include <rclcpp/rclcpp.hpp>
#pragma GCC diagnostic pop

#include <std_msgs/msg/u_int8.hpp>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <string>

#include "monitored_node/monitored_node.hpp"
#include "monitored_node/safety_monitor_interface.hpp"

using namespace std::chrono_literals;

using MonitoredNode = autoware::common::monitored_node::MonitoredNode;
template<class T>
using MonitoredSubscription = autoware::common::monitored_node::MonitoredSubscription<T>;
template<class T>
using MonitoredPublisher = autoware::common::monitored_node::MonitoredPublisher<T>;
using SafetyMonitorInterface = autoware::common::monitored_node::SafetyMonitorInterface;

class MyNode : public MonitoredNode
{
public:
  explicit MyNode(
    std::chrono::milliseconds publishing_min_interval,
    std::chrono::milliseconds publishing_max_interval,
    std::chrono::milliseconds publishing_actual_interval,
    std::chrono::milliseconds subscription_callback_max_delay,
    std::chrono::milliseconds subscription_callback_actual_delay,
    std::vector<std::string> expected_events)
  : MonitoredNode("NODE_NAME"), m_expected_events(expected_events.begin(), expected_events.end())
  {
    // test monitored publisher API
    m_publisher = create_monitored_publisher<std_msgs::msg::UInt8>(
      "TOPIC_NAME", rclcpp::QoS(10), publishing_min_interval, publishing_max_interval);

    // test monitored subscriber API
    m_subscriber = create_monitored_subscription<std_msgs::msg::UInt8>(
      "TOPIC_NAME", rclcpp::QoS(10),
      [this, subscription_callback_actual_delay](const std_msgs::msg::UInt8::SharedPtr msg) {
        std::cout << "subscription callback " <<
          static_cast<uint32_t>(msg->data) << std::endl;
        if (msg->data == 10) {
          msg->data += 1;
          this->m_down_stream_publisher->get_pub_shared_ptr()->publish(*msg);
        }

        rclcpp::sleep_for(subscription_callback_actual_delay);
      },
      subscription_callback_max_delay);

    // create second publisher that uses min/max interval values propagated from the first
    m_down_stream_publisher = create_monitored_publisher<std_msgs::msg::UInt8>(
      "DOWNSTREAM_TOPIC_NAME", rclcpp::QoS(10),
      m_subscriber->get_min_interval_future(),
      m_subscriber->get_max_interval_future());

    // use a second subscriber to check the propagation of max/min interval
    m_down_stream_subscriber =
      create_monitored_subscription<std_msgs::msg::UInt8>(
      "DOWNSTREAM_TOPIC_NAME", rclcpp::QoS(10),
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        std::cout << "DOWN STREAM subscription callback " <<
          static_cast<uint32_t>(msg->data) << std::endl;
        ASSERT_EQ(msg->data, 11);

        if (msg->data == 11) {
          value_propagation_success = true;
        }
      },
      100ms);

    // publish an integer periodically
    m_wall_timer = this->create_wall_timer(
      publishing_actual_interval, [this, publishing_max_interval]() {
        std::cout << "publishing" << std::endl;
        std_msgs::msg::UInt8 msg;
        msg.data = 10;
        this->m_publisher->get_pub_shared_ptr()->publish(msg);

        if (m_subscriber->get_max_interval_future().wait_for(0s) == std::future_status::ready) {
          std::cout << "subscriber interval ready" << std::endl;
        }

        // periodically check if the min/max interval have been propagated correctly
        auto shared_future = m_down_stream_subscriber->get_max_interval_future();
        std::cout << "checking future " << &shared_future << std::endl;
        if (m_down_stream_subscriber->get_max_interval_future().wait_for(0s) ==
        std::future_status::ready)
        {
          std::cout << "m_down_stream_subscriber interval ready" << std::endl;
          auto max_interval =
          m_down_stream_subscriber->get_max_interval_future().get();

          if (max_interval == publishing_max_interval) {
            interval_propagation_success = true;
          }
        }

        // check test success
        std::cout << "is success" << is_success() << std::endl;
        if (is_success() || !rclcpp::ok()) {
          rclcpp::shutdown();
        }
      });

    // Check errors are published to the /diagnostic topic
    m_diagnostic_subscriber = create_monitored_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
      SafetyMonitorInterface::DIAGNOSTIC_TOPIC, rclcpp::QoS(10),
      [&](const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg) {
        std::cout << "received: " << msg->message << std::endl;
        if (m_expected_events.size() > 0 && msg->message == m_expected_events.front()) {
          m_expected_events.erase(m_expected_events.begin());
        }

        std::cout << "expected_event_size: " << m_expected_events.size() << std::endl;
      },
      100ms);
  }

  bool is_success()
  {
    return interval_propagation_success && value_propagation_success &&
           m_expected_events.size() == 0;
  }

private:
  MonitoredPublisher<std_msgs::msg::UInt8>::SharedPtr m_publisher{};
  MonitoredPublisher<std_msgs::msg::UInt8>::SharedPtr m_down_stream_publisher{};

  MonitoredSubscription<std_msgs::msg::UInt8>::SharedPtr m_subscriber{};
  MonitoredSubscription<std_msgs::msg::UInt8>::SharedPtr m_down_stream_subscriber{};
  MonitoredSubscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr
    m_diagnostic_subscriber{};
  rclcpp::TimerBase::SharedPtr m_wall_timer{};
  std::vector<std::string> m_expected_events{};

  bool value_propagation_success{false};
  bool interval_propagation_success{false};
};

class TestSetupTearDown : public ::testing::Test
{
protected:
  void SetUp() {rclcpp::init(0, nullptr);}
  void TearDown() {(void)rclcpp::shutdown();}
};

TEST_F(TestSetupTearDown, SimpleAPITest) {
  std::vector<std::string> expected_events = {
    "TOPIC_NAME:callback_start",
    "TOPIC_NAME:callback_end",
    "DOWNSTREAM_TOPIC_NAME:callback_start",
    "DOWNSTREAM_TOPIC_NAME:callback_end"
  };

  rclcpp::executors::MultiThreadedExecutor exec;
  const auto my_node = std::make_shared<MyNode>(50ms, 100ms, 75ms, 100ms, 50ms, expected_events);
  exec.add_node(my_node);

  while (!my_node->is_success()) {
    exec.spin_some(std::chrono::milliseconds(100LL));
  }
}

TEST_F(TestSetupTearDown, SubscriptionCallbackTooSlowErrorTest) {
  std::vector<std::string> expected_events = {
    "TOPIC_NAME:callback_start",
    "TOPIC_NAME:callback_end:did_not_receive_in_time",
    "TOPIC_NAME:callback_end",
    "DOWNSTREAM_TOPIC_NAME:callback_start",
    "DOWNSTREAM_TOPIC_NAME:callback_end"
  };

  rclcpp::executors::SingleThreadedExecutor exec;
  const auto my_node = std::make_shared<MyNode>(400ms, 500ms, 450ms, 100ms, 200ms, expected_events);
  exec.add_node(my_node);
  exec.spin();

  EXPECT_TRUE(my_node->is_success());
}

TEST_F(TestSetupTearDown, PublishTooFastError) {
  std::vector<std::string> expected_events = {
    "TOPIC_NAME:callback_start",
    "TOPIC_NAME:callback_end",
    "DOWNSTREAM_TOPIC_NAME:callback_start",
    "DOWNSTREAM_TOPIC_NAME:callback_end",
    "TOPIC_NAME:callback_start",
    "TOPIC_NAME:callback_start:arrived_too_soon",
    "TOPIC_NAME:callback_end"
  };

  rclcpp::executors::MultiThreadedExecutor exec;
  const auto my_node = std::make_shared<MyNode>(400ms, 500ms, 300ms, 200ms, 0ms, expected_events);
  exec.add_node(my_node);
  exec.spin();

  EXPECT_TRUE(my_node->is_success());
}

TEST_F(TestSetupTearDown, PublishTooSlowError) {
  std::vector<std::string> expected_events = {
    "TOPIC_NAME:callback_start",
    "TOPIC_NAME:callback_end",
    "DOWNSTREAM_TOPIC_NAME:callback_start",
    "DOWNSTREAM_TOPIC_NAME:callback_end",
    "TOPIC_NAME:callback_start:did_not_receive_in_time",
    "TOPIC_NAME:callback_start",
    "TOPIC_NAME:callback_end"
  };

  rclcpp::executors::MultiThreadedExecutor exec;
  const auto my_node = std::make_shared<MyNode>(400ms, 500ms, 600ms, 200ms, 0ms, expected_events);
  exec.add_node(my_node);
  exec.spin();

  EXPECT_TRUE(my_node->is_success());
}
