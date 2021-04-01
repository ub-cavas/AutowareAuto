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

#ifndef MEASUREMENT_TRANSFORMER_NODES__MEASUREMENT_TRANSFORMER_NODE_BASE_HPP_
#define MEASUREMENT_TRANSFORMER_NODES__MEASUREMENT_TRANSFORMER_NODE_BASE_HPP_

#include <measurement_transformer_nodes/visibility_control.hpp>

#include <common/types.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

using autoware::common::types::bool8_t;
using geometry_msgs::msg::TransformStamped;

namespace autoware
{
namespace measurement_transformer_nodes
{

/// \brief Base class for all measurement transformer node classes
/// \tparam MeasurementT The ROS message type of the measurement
template<class MeasurementT>
class MEASUREMENT_TRANSFORMER_NODES_PUBLIC MeasurementTransformerNode : public rclcpp::Node
{
public:
  /// \brief Default constructor
  /// \param node_name The name of the node to pass on to rclcpp::Node
  /// \param in_topic The name of the input measurement topic
  /// \param out_topic The name of the output measurement topic
  /// \param options An rclcpp::NodeOptions object to pass on to rclcpp::Node
  MeasurementTransformerNode(
    const std::string & node_name,
    const std::string & in_topic,
    const std::string & out_topic,
    const rclcpp::NodeOptions & options)
  : rclcpp::Node(node_name, options)
  {
    m_tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf2_listener = std::make_unique<tf2_ros::TransformListener>(*m_tf2_buffer);

    m_measurement_pub = create_publisher<MeasurementT>(out_topic, rclcpp::QoS{10});
    m_measurement_sub = this->create_subscription<MeasurementT>(
      in_topic, rclcpp::QoS{10},
      std::bind(
        &MeasurementTransformerNode::measurement_callback, this,
        std::placeholders::_1));
  }

protected:
  /// \brief Pure virtual function for processing an incoming measurement
  /// \param measurement The measurement to be transformed
  virtual void measurement_callback(const std::shared_ptr<MeasurementT> measurement) = 0;

  std::unique_ptr<tf2_ros::Buffer> m_tf2_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf2_listener;
  std::shared_ptr<rclcpp::Publisher<MeasurementT>> m_measurement_pub;
  std::shared_ptr<rclcpp::Subscription<MeasurementT>> m_measurement_sub;
};

/// \brief Base class for transformers that affect the child frame of a measurement
/// \tparam MeasurementT The ROS message type of the measurement
template<class MeasurementT>
class MEASUREMENT_TRANSFORMER_NODES_PUBLIC ChildFrameTransformerNode
  : public MeasurementTransformerNode<MeasurementT>
{
public:
  using ParentT = MeasurementTransformerNode<MeasurementT>;

  /// \brief Default constructor
  /// \param node_name The name of the node to pass on to rclcpp::Node
  /// \param in_topic The name of the input measurement topic
  /// \param out_topic The name of the output measurement topic
  /// \param options An rclcpp::NodeOptions object to pass on to rclcpp::Node
  ChildFrameTransformerNode(
    const std::string & node_name,
    const std::string & in_topic,
    const std::string & out_topic,
    const rclcpp::NodeOptions & options)
  : ParentT(
      node_name,
      in_topic,
      out_topic,
      options)
  {
    m_input_child_frame = this->declare_parameter("input_child_frame").template get<std::string>();
    m_output_child_frame =
      this->declare_parameter("output_child_frame").template get<std::string>();
  }

protected:
  /// \brief Pure virtual function to convert a TransformStamped to a measurement
  /// \param tf The TransformStamped to convert to a measurement
  /// \returns A measurement converted from a TransformStamped
  virtual MeasurementT transform_to_measurement(const TransformStamped & tf) = 0;

  /// \brief Pure virtual function to convert a measurement to a TransformStamped
  /// \param measurement The measurement to convert to a TransformStamped
  /// \returns A TransformStamped converted from a measurement
  virtual TransformStamped measurement_to_transform(const MeasurementT & measurement) = 0;

  /// \brief Pure virtual function for applying the transform
  /// \param[in] measurement_in The measurement to be transformed
  /// \param[out] measurement_out The result of the transform
  /// \param[in] tf The transform to apply
  virtual void apply_transform(
    const MeasurementT & measurement_in,
    MeasurementT & measurement_out,
    const TransformStamped & tf) = 0;

  /// \brief Concrete function for processing an incoming measurement
  /// \param measurement The measurement to be transformed
  void measurement_callback(const std::shared_ptr<MeasurementT> measurement) override
  {
    // Get TF between child frames
    TransformStamped child_frame_tf{};

    try {
      child_frame_tf = ParentT::m_tf2_buffer->lookupTransform(
        m_output_child_frame, m_input_child_frame, tf2::getTimestamp(*measurement));
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *(this->get_clock()),
        1000000UL,
        "Transform unavailable - %s",
        ex.what());
      return;
    }

    // Convert child frame TF to measurement
    auto child_frame_measurement = transform_to_measurement(child_frame_tf);

    // Convert incoming measurement to TF
    auto measurement_tf = measurement_to_transform(*measurement);

    // Apply measurement TF to child frame measurement and publish
    MeasurementT out{};
    apply_transform(child_frame_measurement, out, measurement_tf);
    out.header = measurement->header;
    ParentT::m_measurement_pub->publish(out);
  }

  std::string m_input_child_frame{};
  std::string m_output_child_frame{};
};

/// \brief Base class for transformers that affect the parent frame of a measurement
/// \tparam MeasurementT The ROS message type of the measurement
template<class MeasurementT>
class MEASUREMENT_TRANSFORMER_NODES_PUBLIC ParentFrameTransformerNode
  : public MeasurementTransformerNode<MeasurementT>
{
public:
  using ParentT = MeasurementTransformerNode<MeasurementT>;

  /// \brief Default constructor
  /// \param node_name The name of the node to pass on to rclcpp::Node
  /// \param in_topic The name of the input measurement topic
  /// \param out_topic The name of the output measurement topic
  /// \param options An rclcpp::NodeOptions object to pass on to rclcpp::Node
  ParentFrameTransformerNode(
    const std::string & node_name,
    const std::string & in_topic,
    const std::string & out_topic,
    const rclcpp::NodeOptions & options)
  : ParentT(
      node_name,
      in_topic,
      out_topic,
      options)
  {
    m_output_parent_frame =
      this->declare_parameter("output_parent_frame").template get<std::string>();
  }

protected:

  /// \brief Pure virtual function for applying the transform
  /// \param[in] measurement_in The measurement to be transformed
  /// \param[out] measurement_out The result of the transform
  /// \param[in] tf The transform to apply
  virtual void apply_transform(
    const MeasurementT & measurement_in,
    MeasurementT & measurement_out,
    const TransformStamped & tf) = 0;

  /// \brief Concrete function for processing an incoming measurement
  /// \param measurement The measurement to be transformed
  void measurement_callback(const std::shared_ptr<MeasurementT> measurement) override
  {
    TransformStamped tf{};

    try {
      tf = ParentT::m_tf2_buffer->lookupTransform(
        m_output_parent_frame, measurement->header.frame_id, measurement->header.stamp);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *(this->get_clock()),
        1000000UL,
        "Transform unavailable - %s",
        ex.what());
      return;
    }

    MeasurementT out{};
    apply_transform(*measurement, out, tf);
    out.header.stamp = measurement->header.stamp;
    ParentT::m_measurement_pub->publish(out);
  }

  std::string m_input_parent_frame{};
  std::string m_output_parent_frame{};
};

}  // namespace measurement_transformer_nodes
}  // namespace autoware

#endif  // MEASUREMENT_TRANSFORMER_NODES__MEASUREMENT_TRANSFORMER_NODE_BASE_HPP_
