// Copyright 2018 Apex.AI, Inc.
// All rights reserved.

#include <chrono>
#include "cepton_node/cepton_block_node.hpp"


namespace apex_auto
{
namespace drivers
{
namespace cepton_node
{
////////////////////////////////////////////////////////////////////////////////
CeptonBlockNode::CeptonBlockNode(
  const apex::string_strict256_t & node_name,
  const apex::string_strict256_t & topic,
  const apex::string_strict256_t & ip,
  const uint16_t port,
  const std::chrono::nanoseconds timeout_ns,
  const std::chrono::milliseconds init_timeout_ms,
  const std::chrono::nanoseconds max_cycle_time,
  const std::array<float32_t, 3U> & translation,
  const std::array<float32_t, 4U> & rotation,
  const std::size_t expected_num_subscribers,
  const apex::string_strict256_t & node_namespace)
: UdpDriverNode<Packet, apex_auto_msgs::msg::PointBlock>(
    node_name,
    topic,
    ip,
    port,
    timeout_ns,
    init_timeout_ms,
    max_cycle_time,
    expected_num_subscribers,
    node_namespace),
  m_transform(cepton_sdk::util::CompiledTransform::create(translation.data(), rotation.data()))
{
}
////////////////////////////////////////////////////////////////////////////////
CeptonBlockNode::CeptonBlockNode(
  const apex::string_strict256_t & node_name,
  const std::chrono::nanoseconds max_cycle_time,
  const apex::string_strict256_t & node_namespace)
: UdpDriverNode(node_name, max_cycle_time, node_namespace)
{
  std::array<float32_t, 3U> translation;
  translation[0U] = static_cast<float32_t>(get_parameter("transform.dx_m").as_double());
  translation[1U] = static_cast<float32_t>(get_parameter("transform.dy_m").as_double());
  translation[2U] = static_cast<float32_t>(get_parameter("transform.dz_m").as_double());
  std::array<float32_t, 4U> rotation;
  rotation[0U] = static_cast<float32_t>(get_parameter("transform.qx").as_double());
  rotation[1U] = static_cast<float32_t>(get_parameter("transform.qy").as_double());
  rotation[2U] = static_cast<float32_t>(get_parameter("transform.qz").as_double());
  rotation[3U] = static_cast<float32_t>(get_parameter("transform.qw").as_double());
  m_transform =
    cepton_sdk::util::CompiledTransform::create(translation.data(), rotation.data());
}

////////////////////////////////////////////////////////////////////////////////
CeptonBlockNode::~CeptonBlockNode()
{
  (void)cepton_sdk_deinitialize();
}

////////////////////////////////////////////////////////////////////////////////
//lint -e9175 Needed to match parent API NOLINT
void CeptonBlockNode::init_output(apex_auto_msgs::msg::PointBlock & output)
{
  cepton_init(output);
}

////////////////////////////////////////////////////////////////////////////////
bool8_t CeptonBlockNode::convert(const Packet & pkt, apex_auto_msgs::msg::PointBlock & output)
{
  // unix time (us)
  const int64_t timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  // I am assuming the listen_points callback gets called inside mock_network_receive
  constexpr cepton_sdk::SensorHandle dummy_handle = 0U;
  const cepton_sdk::SensorError ret =
    cepton_sdk::mock_network_receive(dummy_handle, timestamp, &pkt.data[0U], CEPTON_PACKET_SIZE);
  if (static_cast<int32_t>(CEPTON_SUCCESS) != static_cast<cepton_sdk::SensorErrorCode>(ret)) {
    throw ret;
  }
  // callback should have been called with &output as user data
  // Apply transform
  for (auto & pt : output.points) {
    m_transform.apply(pt.x, pt.y, pt.z);
    pt.id = 0U;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool8_t CeptonBlockNode::get_output_remainder(apex_auto_msgs::msg::PointBlock & output)
{
  (void)output;
  return false;
}
}  // namespace cepton_node
}  // namespace drivers
}  // namespace apex_auto
