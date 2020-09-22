// Copyright 2020 Apex.AI, Inc.
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

/// \copyright Copyright 2020 Apex.AI, Inc.
/// All rights reserved.

#include <state_estimation_node/state_estimation_node.hpp>

#include <state_estimation_node/time.hpp>
#include <state_estimation_node/measurement_conversion.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <string>
#include <vector>
#include <memory>
#include <limits>
#include <functional>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;

namespace
{
constexpr int kDefaultHistory{10};  // TODO(igor): remove this.
constexpr float64_t kInvalidFrequency{-1.0};  // Frames per second.
const std::chrono::milliseconds kDefaultTimeBetweenUpdates{100LL};
const char kDefaultOutputTopic[]{"filtered_state"};
const char kConstantAccelerationTag[]{"ConstantAcceleration"};
const char kConstantAcceleration3DTag[]{"ConstantAcceleration3D"};
constexpr auto kCovarianceMatrixRows{6U};
constexpr auto kIndexX{0U};
constexpr auto kIndexY{kCovarianceMatrixRows + 1U};
constexpr auto kCovarianceMatrixRowsSquared{kCovarianceMatrixRows * kCovarianceMatrixRows};
static_assert(std::tuple_size<
    geometry_msgs::msg::PoseWithCovariance::_covariance_type>::value ==
  kCovarianceMatrixRowsSquared, "We expect the covariance matrix to have 36 entries.");

/// Convert the ROS timestamp to chrono time point.
autoware::prediction::GlobalTime
to_time_point(const rclcpp::Time & time)
{
  return autoware::prediction::GlobalTime{
    std::chrono::system_clock::time_point{std::chrono::nanoseconds{time.nanoseconds()}}};
}

void assert_all_entries_positive(const std::vector<float64_t> & entries, const std::string & tag)
{
  for (const auto entry : entries) {
    if (std::isnan(entry) || std::isinf(entry) || entry <= 0) {
      throw std::runtime_error(tag + ": entries must all be positive.");
    }
  }
}

std::chrono::nanoseconds validate_publish_frequency(
  float64_t publish_frequency,
  bool8_t publish_data_driven)
{
  std::chrono::nanoseconds time_between_publish_requests;
  if ((publish_frequency > 0.0) && publish_data_driven) {
    throw std::logic_error(
            "Please provide either 'output_frequency' setting or 'data_driven' one, not both.");
  } else if (publish_data_driven) {
    time_between_publish_requests = kDefaultTimeBetweenUpdates;
  } else if (publish_frequency > 0.0) {
    const float64_t nanoseconds_in_second{
      std::chrono::nanoseconds{std::chrono::seconds{1}}.count()};
    time_between_publish_requests = std::chrono::nanoseconds{
      static_cast<std::uint64_t>(std::floor(nanoseconds_in_second / publish_frequency))};
  } else {
    throw std::logic_error(
            "Please provide either 'output_frequency' setting or 'data_driven' one.");
  }
  return time_between_publish_requests;
}

template<typename MotionModelT, std::int32_t kStateDim, std::int32_t kNoiseDim>
Eigen::Matrix<float32_t, kStateDim, kNoiseDim> create_process_noise_variances(
  const std::vector<float64_t> & position_variance,
  const std::vector<float64_t> & velocity_variance,
  const std::vector<float64_t> & acceleration_variance)
{
  if (acceleration_variance.size() != static_cast<size_t>(kNoiseDim)) {
    throw std::logic_error("For now we require a 2D acceleration variance.");
  }
  Eigen::Matrix<float32_t, kStateDim, kNoiseDim> process_noise_variances{
    Eigen::Matrix<float32_t, kStateDim, kNoiseDim>::Zero()};
  if (position_variance.size() == static_cast<size_t>(kNoiseDim)) {
    assert_all_entries_positive(position_variance, "position_variance");
    const auto first_pose_idx = MotionModelT::States::POSE_X;
    for (auto i = 0; i < kNoiseDim; ++i) {
      process_noise_variances(first_pose_idx + i, i) =
        static_cast<float32_t>(position_variance[static_cast<std::size_t>(i)]);
    }
  }
  if (velocity_variance.size() == static_cast<size_t>(kNoiseDim)) {
    assert_all_entries_positive(velocity_variance, "velocity_variance");
    const auto first_velocity_idx = MotionModelT::States::VELOCITY_X;
    for (auto i = 0; i < kNoiseDim; ++i) {
      process_noise_variances(first_velocity_idx + i, i) =
        static_cast<float32_t>(velocity_variance[static_cast<std::size_t>(i)]);
    }
  }
  assert_all_entries_positive(acceleration_variance, "acceleration_variance");
  const auto first_acceleration_idx = MotionModelT::States::ACCELERATION_X;
  for (auto i = 0; i < kNoiseDim; ++i) {
    process_noise_variances(first_acceleration_idx + i, i) =
      static_cast<float32_t>(acceleration_variance[static_cast<std::size_t>(i)]);
  }
  return process_noise_variances;
}

template<std::int32_t kStateDim>
Eigen::Matrix<float32_t, kStateDim, kStateDim> create_state_variances(
  const std::vector<float64_t> & state_variances)
{
  if (state_variances.size() != static_cast<size_t>(kStateDim)) {
    throw std::logic_error("State variances are of wrong size.");
  }
  assert_all_entries_positive(state_variances, "state_variances");
  Eigen::Matrix<float32_t, kStateDim, 1> diagonal;
  for (auto i = 0U; i < state_variances.size(); ++i) {
    diagonal[i] = static_cast<float32_t>(state_variances[i]);
  }
  return diagonal.asDiagonal();
}

autoware::common::types::float64_t get_speed(const geometry_msgs::msg::Twist & twist)
{
  const auto x = twist.linear.x;
  const auto y = twist.linear.y;
  return std::sqrt(x * x + y * y);
}


}  // namespace

namespace autoware
{
namespace prediction
{
namespace state_estimation_node
{

StateEstimationNode::StateEstimationNode(
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node{"state_estimation_node", node_options},
  m_tf_listener(m_tf_buffer, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false)
{
  m_frame_id = declare_parameter("frame_id").get<std::string>();
  m_child_frame_id = declare_parameter("child_frame_id").get<std::string>();
  m_publish_frequency = declare_parameter("output_frequency", kInvalidFrequency);
  m_publish_data_driven = declare_parameter("data_driven", false);
  const auto time_between_publish_requests{
    validate_publish_frequency(m_publish_frequency, m_publish_data_driven)};
  if (!m_publish_data_driven) {
    m_wall_timer = create_wall_timer(
      time_between_publish_requests,
      std::bind(&StateEstimationNode::predict_and_publish_current_state, this));
  }

  const auto position_variance{
    declare_parameter("process_noise_variances.position", std::vector<float64_t>{})};
  const auto velocity_variance{
    declare_parameter("process_noise_variances.velocity", std::vector<float64_t>{})};
  const auto acceleration_variance{
    declare_parameter("process_noise_variances.acceleration", std::vector<float64_t>{})};
  const auto state_variances{
    declare_parameter("state_variances", std::vector<float64_t>{})};
  const auto mahalanobis_threshold{
    declare_parameter("mahalanobis_threshold", std::numeric_limits<float32_t>::max())};

  const auto motion_model_name{
    declare_parameter("motion_model", "ConstantAcceleration")};

  if (motion_model_name == kConstantAccelerationTag) {
    m_ekf = ConstantAccelerationFilter{
      create_state_variances<ConstantAccelerationFilter::NumOfStates>(state_variances),
      create_process_noise_variances<
        ConstantAccelerationFilter::MotionModel,
        ConstantAccelerationFilter::NumOfStates,
        ConstantAccelerationFilter::ProcessNoiseDim>(
        position_variance, velocity_variance, acceleration_variance),
      time_between_publish_requests,
      m_frame_id,
      static_cast<float>(mahalanobis_threshold)};
  } else if (motion_model_name == kConstantAcceleration3DTag) {
    m_ekf = ConstantAccelerationFilter3D{
      create_state_variances<ConstantAccelerationFilter3D::NumOfStates>(state_variances),
      create_process_noise_variances<
        ConstantAccelerationFilter3D::MotionModel,
        ConstantAccelerationFilter3D::NumOfStates,
        ConstantAccelerationFilter3D::ProcessNoiseDim>(
        position_variance, velocity_variance, acceleration_variance),
      time_between_publish_requests,
      m_frame_id,
      static_cast<float>(mahalanobis_threshold)};
  }


  const std::vector<std::string> empty_vector{};
  const auto input_odom_topics{declare_parameter("topics.input_odom", empty_vector)};
  const auto input_pose_topics{declare_parameter("topics.input_pose", empty_vector)};
  const auto input_twist_topics{declare_parameter("topics.input_twist", empty_vector)};
  if (input_odom_topics.empty() && input_pose_topics.empty() && input_twist_topics.empty()) {
    throw std::runtime_error("No input topics provided. Make sure to set these in the param file.");
  }
  create_subscriptions<OdomMsgT>(
    input_odom_topics, &m_odom_subscribers, &StateEstimationNode::odom_callback);
  create_subscriptions<PoseMsgT>(
    input_pose_topics, &m_pose_subscribers, &StateEstimationNode::pose_callback);
  create_subscriptions<TwistMsgT>(
    input_twist_topics, &m_twist_subscribers, &StateEstimationNode::twist_callback);

  m_publisher = create_publisher<nav_msgs::msg::Odometry>(kDefaultOutputTopic, kDefaultHistory);

  const auto publish_ft = declare_parameter("publish_tf", false);
  if (publish_ft) {
    m_tf_publisher = create_publisher<tf2_msgs::msg::TFMessage>("/tf", kDefaultHistory);
  }

  m_min_speed_to_use_speed_orientation = declare_parameter(
    "min_speed_to_use_speed_orientation", 0.0);
}

void StateEstimationNode::odom_callback(const OdomMsgT::SharedPtr msg)
{
  struct ObservationVisitor
  {
    void operator()(ConstantAccelerationFilter & ekf) const
    {
      ekf.observation_update(
        time_observation_received,
        message_to_measurement<autoware::prediction::MeasurementPoseAndSpeed>(
          *msg, tf2::transformToEigen(transform).template cast<float32_t>()));
    }
    void operator()(autoware::prediction::ConstantAccelerationFilter3D & ekf) const
    {
      ekf.observation_update(
        time_observation_received,
        message_to_measurement<MeasurementPoseAndSpeed3D>(
          *msg, tf2::transformToEigen(transform).template cast<float32_t>()));
    }
    void operator()(mpark::monostate &) const {}

    autoware::prediction::GlobalTime time_observation_received;
    const OdomMsgT::SharedPtr msg;
    geometry_msgs::msg::TransformStamped transform;
  };

  const ObservationVisitor visitor{to_time_point(now()), msg, get_transform(msg->header)};
  mpark::visit(visitor, m_ekf);
  geometry_msgs::msg::QuaternionStamped orientation_in_expected_frame;
  tf2::doTransform(
    geometry_msgs::msg::QuaternionStamped{}.
    set__quaternion(msg->pose.pose.orientation).
    set__header(msg->header),
    orientation_in_expected_frame, visitor.transform);
  update_latest_orientation_if_needed(orientation_in_expected_frame);
  if (m_publish_data_driven) {
    publish_current_state();
  }
}

void StateEstimationNode::pose_callback(const PoseMsgT::SharedPtr msg)
{
  struct ObservationVisitor
  {
    void operator()(ConstantAccelerationFilter & ekf) const
    {
      ekf.observation_update(
        time_observation_received,
        message_to_measurement<autoware::prediction::MeasurementPose>(
          *msg, tf2::transformToEigen(transform).template cast<float32_t>()));
    }
    void operator()(autoware::prediction::ConstantAccelerationFilter3D & ekf) const
    {
      ekf.observation_update(
        time_observation_received,
        message_to_measurement<MeasurementPose3D>(
          *msg, tf2::transformToEigen(transform).template cast<float32_t>()));
    }
    void operator()(mpark::monostate &) const {}

    autoware::prediction::GlobalTime time_observation_received;
    const PoseMsgT::SharedPtr msg;
    geometry_msgs::msg::TransformStamped transform;
  };

  const ObservationVisitor visitor{to_time_point(now()), msg, get_transform(msg->header)};
  mpark::visit(visitor, m_ekf);
  geometry_msgs::msg::QuaternionStamped orientation_in_expected_frame;
  tf2::doTransform(
    geometry_msgs::msg::QuaternionStamped{}.
    set__quaternion(msg->pose.pose.orientation).
    set__header(msg->header),
    orientation_in_expected_frame, visitor.transform);
  update_latest_orientation_if_needed(orientation_in_expected_frame);
  if (m_publish_data_driven) {
    publish_current_state();
  }
}

void StateEstimationNode::twist_callback(const TwistMsgT::SharedPtr msg)
{
  struct ObservationVisitor
  {
    void operator()(ConstantAccelerationFilter & ekf) const
    {
      if (!ekf.is_initialized()) {
        return;
      }
      ekf.observation_update(
        time_observation_received,
        message_to_measurement<autoware::prediction::MeasurementSpeed>(
          *msg, tf2::transformToEigen(transform).template cast<float32_t>()));
    }
    void operator()(autoware::prediction::ConstantAccelerationFilter3D & ekf) const
    {
      if (!ekf.is_initialized()) {
        return;
      }
      ekf.observation_update(
        time_observation_received,
        message_to_measurement<MeasurementSpeed3D>(
          *msg, tf2::transformToEigen(transform).template cast<float32_t>()));
    }
    void operator()(mpark::monostate &) const {}

    autoware::prediction::GlobalTime time_observation_received;
    const TwistMsgT::SharedPtr msg;
    geometry_msgs::msg::TransformStamped transform;
  };

  const ObservationVisitor visitor{to_time_point(now()), msg, get_transform(msg->header)};
  mpark::visit(visitor, m_ekf);
  if (m_publish_data_driven) {
    publish_current_state();
  }
}

geometry_msgs::msg::TransformStamped StateEstimationNode::get_transform(
  const std_msgs::msg::Header & header)
{
  // Get the transform between the msg and the output frame. We treat the
  // possible exceptions as unrecoverable and let them bubble up.
  return m_tf_buffer.lookupTransform(m_frame_id, header.frame_id, to_time_point(header.stamp));
}

void StateEstimationNode::predict_and_publish_current_state()
{
  struct PredictionVisitor
  {
    void operator()(ConstantAccelerationFilter & ekf) const
    {
      if (!ekf.is_initialized()) {
        return;
      }
      ekf.temporal_update(time_now);
    }
    void operator()(autoware::prediction::ConstantAccelerationFilter3D & ekf) const
    {
      if (!ekf.is_initialized()) {
        return;
      }
      ekf.temporal_update(time_now);
    }
    void operator()(mpark::monostate &) const {}

    autoware::prediction::GlobalTime time_now;
  };
  PredictionVisitor visitor{to_time_point(now())};
  mpark::visit(visitor, m_ekf);
}

void StateEstimationNode::publish_current_state()
{
  struct GetStateVisitor
  {
    void operator()(ConstantAccelerationFilter & ekf)
    {
      if (!ekf.is_initialized()) {
        return;
      }
      state = ekf.get_state();
      state_set = true;
    }
    void operator()(autoware::prediction::ConstantAccelerationFilter3D & ekf)
    {
      if (!ekf.is_initialized()) {
        return;
      }
      state = ekf.get_state();
      state_set = true;
    }
    void operator()(mpark::monostate &) const {}

    bool state_set{};
    nav_msgs::msg::Odometry state;
  };
  if (!m_publisher) {return;}
  GetStateVisitor visitor;
  mpark::visit(visitor, m_ekf);
  if (!visitor.state_set) {return;}
  auto state = visitor.state;
  if (get_speed(state.twist.twist) < m_min_speed_to_use_speed_orientation) {
    state.pose.pose.orientation = m_latest_orientation.quaternion;
  }
  m_publisher->publish(state);
  if (m_tf_publisher) {
    TfMsgT tf_msg{};
    tf_msg.transforms.emplace_back();
    auto & tf = tf_msg.transforms.back();
    tf.header = state.header;
    tf.child_frame_id = m_child_frame_id;
    tf.transform.translation.x = state.pose.pose.position.x;
    tf.transform.translation.y = state.pose.pose.position.y;
    tf.transform.translation.z = state.pose.pose.position.z;
    tf.transform.rotation = state.pose.pose.orientation;
    m_tf_publisher->publish(tf_msg);
  }
}

void StateEstimationNode::update_latest_orientation_if_needed(
  const geometry_msgs::msg::QuaternionStamped & rotation)
{
  if (m_latest_orientation.header.stamp.sec > rotation.header.stamp.sec) {return;}
  if (m_latest_orientation.header.stamp.sec == rotation.header.stamp.sec) {
    if (m_latest_orientation.header.stamp.nanosec > rotation.header.stamp.nanosec) {return;}
  }
  m_latest_orientation = rotation;
}

template<typename MessageT>
void StateEstimationNode::create_subscriptions(
  const std::vector<std::string> & input_topics,
  std::vector<typename rclcpp::Subscription<MessageT>::SharedPtr> * subscribers,
  CallbackFnT<MessageT> callback)
{
  for (const auto & input_topic : input_topics) {
    subscribers->emplace_back(
      create_subscription<MessageT>(
        input_topic, kDefaultHistory,
        std::bind(callback, this, std::placeholders::_1)));
  }
}

template void StateEstimationNode::create_subscriptions<StateEstimationNode::OdomMsgT>(
  const std::vector<std::string> &,
  std::vector<rclcpp::Subscription<OdomMsgT>::SharedPtr> *,
  CallbackFnT<StateEstimationNode::OdomMsgT>);
template void StateEstimationNode::create_subscriptions<StateEstimationNode::PoseMsgT>(
  const std::vector<std::string> &,
  std::vector<rclcpp::Subscription<PoseMsgT>::SharedPtr> *,
  CallbackFnT<StateEstimationNode::PoseMsgT>);
template void StateEstimationNode::create_subscriptions<StateEstimationNode::TwistMsgT>(
  const std::vector<std::string> &,
  std::vector<rclcpp::Subscription<TwistMsgT>::SharedPtr> *,
  CallbackFnT<StateEstimationNode::TwistMsgT>);


}  // namespace state_estimation_node
}  // namespace prediction
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::prediction::state_estimation_node::StateEstimationNode)
