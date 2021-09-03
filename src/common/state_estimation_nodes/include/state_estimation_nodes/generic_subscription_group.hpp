// Copyright 2021 the Autoware Foundation
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
//
// Developed by Apex.AI, Inc.

/// \copyright Copyright 2021 the Autoware Foundation
/// All rights reserved.
/// \file

#ifndef STATE_ESTIMATION_NODES__UTILS_HPP_
#define STATE_ESTIMATION_NODES__UTILS_HPP_

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <autoware_auto_msgs/msg/classified_roi_array.hpp>
#include <rclcpp/subscription.hpp>

namespace autoware
{
namespace common
{
namespace state_estimation
{
namespace details
{

template<typename T>
struct MessageTypeString
{
//  static_assert(common::type_traits::impossible_branch<T>(),
//  "Trying to use an unsupported message type");
//  static constexpr auto value = "Invalid";
};

template<>
struct MessageTypeString<sensor_msgs::msg::PointCloud2>
{
  static constexpr auto value = "pointcloud2";
};

template<>
struct MessageTypeString<sensor_msgs::msg::Image>
{
  static constexpr auto value = "image";
};

template<>
struct MessageTypeString<autoware_auto_msgs::msg::ClassifiedRoiArray>
{
  static constexpr auto value = "classified_rois";
};

constexpr char const * MessageTypeString<sensor_msgs::msg::PointCloud2>::value;
constexpr char const * MessageTypeString<sensor_msgs::msg::Image>::value;
constexpr char const * MessageTypeString<autoware_auto_msgs::msg::ClassifiedRoiArray>::value;

using TriggerTypesTuple = std::tuple<
  sensor_msgs::msg::PointCloud2,
  sensor_msgs::msg::Image,
  autoware_auto_msgs::msg::ClassifiedRoiArray
>;
}

struct STATE_ESTIMATION_NODES_LOCAL GenericSubscriptionConfig
{
  GenericSubscriptionConfig(
    const std::string & type_string, const std::string & topic,
    const rclcpp::QoS qos)
  : m_type_string{type_string}, m_topic{topic}, m_qos{qos}
  {
    std::transform(
      m_type_string.begin(), m_type_string.end(), m_type_string.begin(),
      [](auto c) {return std::tolower(c);});
  }
  const std::string & type_string() const noexcept {return m_type_string;}
  const std::string & topic() const noexcept {return m_topic;}
  const rclcpp::QoS & qos() const noexcept {return m_qos;}

private:
  std::string m_type_string;
  std::string m_topic;
  rclcpp::QoS m_qos;
};

template<typename MessageTypesTupleT>
class GenericSubscriptionGroup;

template<typename ... MessageTypes>
class STATE_ESTIMATION_NODES_LOCAL GenericSubscriptionGroup<std::tuple<MessageTypes...>>
{
public:
  template<typename FunctorT>
  GenericSubscriptionGroup(
    rclcpp::Node & node_ref,
    const std::vector<GenericSubscriptionConfig> & sub_configs,
    const FunctorT & generic_callback)
  {
    for (const auto & sub_config : sub_configs) {
      const auto tuple_visitor =
        [this, &sub_config, &node_ref, &generic_callback](const auto & dummy_msg) {
          this->try_register_subscription<std::decay_t<decltype(dummy_msg)>>(
            node_ref, sub_config, generic_callback);
        };
      // Use a tuple visitor to visit each message type until a type that has a matching
      // `type_string` is found. New subscriptions will only be created for matching types.
      common::type_traits::visit(std::tuple<MessageTypes...>{}, tuple_visitor);
    }
  }

private:
  using SubscriptionVariant =
    mpark::variant<typename rclcpp::Subscription<MessageTypes>::SharedPtr...>;

  template<typename MsgType, typename FunctorT>
  void try_register_subscription(
    rclcpp::Node & node_ref,
    const GenericSubscriptionConfig & config, const FunctorT & callback)
  {
    if (config.type_string() == details::MessageTypeString<MsgType>::value) {
      m_subs.emplace_back(
        node_ref.create_subscription<MsgType>(
          config.topic(), config.qos(),
          [&callback](typename MsgType::ConstSharedPtr msg) {callback(msg);}));
    }
  }

  std::vector<SubscriptionVariant> m_subs;
};

using TriggerSubscriptionGroup = GenericSubscriptionGroup<details::TriggerTypesTuple>;

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware


#endif  // STATE_ESTIMATION_NODES__UTILS_HPP_
