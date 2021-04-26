// Copyright 2021 The Autoware Foundation
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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the synchronized_subscription_node class.

#ifndef SYNCHRONIZED_SUBSCRIPTION__SYNCHRONIZED_SUBSCRIPTION_HPP_
#define SYNCHRONIZED_SUBSCRIPTION__SYNCHRONIZED_SUBSCRIPTION_HPP_

#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "synchronized_subscription/visibility_control.hpp"

namespace autoware
{
namespace common
{

/// \class DroppedMessageReason
/// \brief Explains why a message was dropped.
enum class DroppedMessageReason
{
  /// A secondary message that was newer than the primary message's timestamp arrived.
  SecondaryMsgSkipped,
  /// Two primary messages arrived without any secondary message inbetween.
  PrimaryMsgAlreadyBuffered,
};

/// \class SynchronizedSubscriptionConfig
/// \brief Contains callbacks and options.
template<typename PrimaryMsg, typename SecondaryMsg>
struct SYNCHRONIZED_SUBSCRIPTION_PUBLIC SynchronizedSubscriptionConfig
{
  using PrimaryPtr = std::shared_ptr<PrimaryMsg const>;
  using SecondaryPtr = std::shared_ptr<SecondaryMsg const>;
  /// The joint callback.
  std::function<void(PrimaryPtr, SecondaryPtr)> cb;
  /// Callback for missing secondary message, receives the dropped message.
  std::function<void(PrimaryPtr, DroppedMessageReason)> dropped_msg_cb;
};

/// \class SynchronizedSubscription
/// \brief A subscription for two topics whose messages have matching timestamps.
/// It is assumed that messages are delivered in order wrt other messages on the same topic.
/// It is assumed that no two successive primary messages arrive before a corresponding secondary
/// message arrives.
template<typename PrimaryMsg, typename SecondaryMsg>
class SYNCHRONIZED_SUBSCRIPTION_PUBLIC SynchronizedSubscription
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  SynchronizedSubscription(
    const std::string & primary_topic, const rclcpp::QoS & primary_qos,
    const std::string & secondary_topic, const rclcpp::QoS & secondary_qos,
    rclcpp::Node & node,
    SynchronizedSubscriptionConfig<PrimaryMsg, SecondaryMsg> config)
  : m_primary_sub(node.create_subscription(primary_topic, primary_qos,
      [this](const auto msg) {this->primary_callback(msg);})),
    m_secondary_sub(
      node.create_subscription(
        secondary_topic, secondary_qos,
        [this](const auto msg) {this->secondary_callback(msg);})),
    m_config(std::move(config))
  {}

private:
  void primary_callback(std::shared_ptr<PrimaryMsg const> msg)
  {
    // If there is already a queued primary message, that violates the assumptions.
    if (m_primary_buffer) {
      // Drop the older message. It should be the one in the buffer, but let's check.
      if (m_primary_buffer->header.stamp < msg->header.stamp) {
        // Move the to-be-dropped message into msg.
        msg.swap(m_primary_buffer);
      }
      if (m_config.dropped_msg_cb) {
        m_config.dropped_msg_cb(std::move(msg), DroppedMessageReason::PrimaryMsgAlreadyBuffered);
      }
      // m_secondary_buffer is empty already by the invariant, so no need to adjust anything.
      m_time_cutoff = m_primary_buffer->header.time;
      return;
    }

    // Check if there is already a matching or newer secondary message.
    const rclcpp::Time msg_time = msg->header.stamp;
    m_time_cutoff = msg_time;
    const auto newer_or_equal_snd_msg = std::find_if(
      m_secondary_buffer.cbegin(), m_secondary_buffer.cend(), [&msg_time](const auto & snd_msg) {
        // Messages are stored old to new, so this will return the correct message.
        return rclcpp::Time(snd_msg->header.stamp) >= msg_time;
      });
    if (newer_or_equal_snd_msg == m_secondary_buffer.cend()) {
      // There is no matching or newer secondary message. That means we need to queue up the
      // primary message for later processing. All existing messages are outdated, so they can be
      // deleted.
      m_secondary_buffer.clear();
      m_primary_buffer = std::move(msg);
      return;
    }
    if (newer_or_equal_snd_msg->header.stamp == msg->header.stamp) {
      // Happy path! Process the message pair.
      m_config.cb(std::move(msg), *newer_or_equal_snd_msg);
      // Delete all odom messages older than msg. They are in the front since secondary message
      // are ordered old to new. The current iterator is incremented, since it needs to be erased
      // too.
      m_secondary_buffer.erase(m_secondary_buffer.cbegin(), ++newer_or_equal_snd_msg);
    } else {
      // There is no matching secondary message, but a newer one (since newer_or_equal_snd_msg is
      // not the end). Since the secondary messages are assumed to arrive in order, this primary
      // message is determined to not have a matching secondary message and is dropped.
      m_config.dropped_msg_cb(std::move(msg), DroppedMessageReason::SecondaryMsgSkipped);
      // Delete all odom messages older than msg.
      m_secondary_buffer.erase(m_secondary_buffer.cbegin(), newer_or_equal_snd_msg);
    }
  }

  void secondary_callback(std::shared_ptr<SecondaryMsg const> msg)
  {
    const rclcpp::Time msg_time = msg->header.stamp;
    if (m_primary_buffer) {
      // Compare the secondary message's timestamp with the queued primary message's.
      const rclcpp::Time prm_msg_time = m_primary_buffer->header.stamp;
      if (msg_time == prm_msg_time) {
        // Happy path! Process the message pair.
        m_config.cb(std::move(m_primary_buffer), std::move(msg));
        m_primary_buffer = nullptr;
        // By the invariant, m_secondary_buffer is already empty.
      } else if (msg_time > prm_msg_time) {
        // Since the secondary messages are assumed to arrive in order, this primary
        // message is determined to not have a matching secondary message and is dropped.
        m_config.dropped_msg_cb(
          std::move(m_primary_buffer), DroppedMessageReason::SecondaryMsgSkipped);
        m_primary_buffer = nullptr;
        // The secondary message might match the next primary message, so needs to be stored.
        m_secondary_buffer.push_back(std::move(msg));
      } else {
        // The secondary message is older than the queued message and is discarded.
        // No action needed.
      }
      return;
    }

    // Add the message to the secondary buffer, but maintain orderedness.
    if (m_secondary_buffer.empty()) {
      // A buffer with one element is trivially ordered, so that is fine.
      // But to maintain the invariant that it will only contain messages newer than the previous
      // primary message, another check is needed.
      if (msg_time > m_time_cutoff) {
        m_secondary_buffer.push_back(std::move(msg));
      }
    } else if (rclcpp::Time(m_secondary_buffer.last()->header.stamp) < msg_time) {
      // The message is indeed newer than the previous one, so pushing it into the buffer will
      // maintain orderedness.
      m_secondary_buffer.push_back(std::move(msg));
    }
  }

  /// Subscription to primary, triggering messages.
  typename rclcpp::Subscription<PrimaryMsg>::SharedPtr m_primary_sub;
  /// Subscription to secondary, auxiliary messages.
  typename rclcpp::Subscription<SecondaryMsg>::SharedPtr m_secondary_sub;
  /// A buffer for the primary message. If it is set, m_secondary_buffer will be empty.
  std::shared_ptr<SecondaryMsg const> m_primary_buffer;
  /// A buffer for the secondary messages. It will only contain messages newer than the previous
  /// primary message and is ordered old to new.
  std::deque<std::shared_ptr<SecondaryMsg const>> m_secondary_buffer;
  /// Config containing options and callbacks.
  SynchronizedSubscriptionConfig<PrimaryMsg, SecondaryMsg> m_config;
  /// Time of the previous primary message, or 0.
  rclcpp::Time m_time_cutoff = {0, 0};
};
}  // namespace common
}  // namespace autoware

#endif  // SYNCHRONIZED_SUBSCRIPTION__SYNCHRONIZED_SUBSCRIPTION_HPP_
