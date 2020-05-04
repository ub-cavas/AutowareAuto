// Copyright 2018 Apex.AI, Inc.
// All rights reserved.

#include "cepton_node/cepton_cloud_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <lidar_utils/point_cloud_utils.hpp>

namespace apex_auto
{
namespace drivers
{
namespace cepton_node
{
// Check consistency of cepton point with our point cloud spec
static_assert(std::is_same<decltype(cepton_sdk::util::SensorPoint::intensity), float32_t>::value,
  "cepton SensorPoint::Intensity is not float32_t");
static_assert(std::is_same<decltype(cepton_sdk::util::SensorPoint::x), float32_t>::value,
  "cepton SensorPoint::x is not float32_t");
static_assert(std::is_same<decltype(cepton_sdk::util::SensorPoint::y), float32_t>::value,
  "cepton SensorPoint::y is not float32_t");
static_assert(std::is_same<decltype(cepton_sdk::util::SensorPoint::z), float32_t>::value,
  "cepton SensorPoint::z is not float32_t");
////////////////////////////////////////////////////////////////////////////////
CeptonCloudNode::CeptonCloudNode(
  const apex::string_strict256_t & node_name,
  const apex::string_strict256_t & topic,
  const apex::string_strict256_t & ip,
  const uint16_t port,
  const std::chrono::nanoseconds timeout_ns,
  const std::chrono::milliseconds init_timeout_ms,
  const std::chrono::nanoseconds max_cycle_time,
  const uint32_t cloud_size,
  const apex::string_strict256_t & frame_id,
  const std::array<float32_t, 3U> & translation,
  const std::array<float32_t, 4U> & rotation,
  const std::size_t expected_num_subscribers,
  const apex::string_strict256_t & node_namespace)
: UdpDriverNode<Packet, sensor_msgs::msg::PointCloud2>(
    node_name,
    topic,
    ip,
    port,
    timeout_ns,
    init_timeout_ms,
    max_cycle_time,
    expected_num_subscribers,
    node_namespace),
  m_cloud_size(cloud_size),
  m_frame_id(frame_id),
  m_transform(cepton_sdk::util::CompiledTransform::create(translation.data(), rotation.data()))
{
}

////////////////////////////////////////////////////////////////////////////////
CeptonCloudNode::CeptonCloudNode(
  const apex::string_strict256_t & node_name,
  const std::chrono::nanoseconds max_cycle_time,
  const apex::string_strict256_t & node_namespace)
: UdpDriverNode(node_name, max_cycle_time, node_namespace),
  m_cloud_size(static_cast<uint32_t>(get_parameter("cloud_size").as_int())),
  m_frame_id(get_parameter("frame_id").as_string().c_str())
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
CeptonCloudNode::~CeptonCloudNode()
{
  (void)cepton_sdk_deinitialize();
}

////////////////////////////////////////////////////////////////////////////////
void CeptonCloudNode::init_output(sensor_msgs::msg::PointCloud2 & output)
{
  cepton_init(output);
  common::lidar_utils::init_pcl_msg(output, m_frame_id, m_cloud_size);
  check_fields(output);
}

////////////////////////////////////////////////////////////////////////////////
bool8_t CeptonCloudNode::convert(const Packet & pkt, sensor_msgs::msg::PointCloud2 & output)
{
  // Output is assumed to have been registered with the callback
  output.data.clear();
  output.width = 0U;
  // unix time (us)
  const auto now = apex::system_clock::now();
  const int64_t timestamp_us =
    std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
  output.header.stamp = apex::to_msg_time(now);
  // I am assuming the listen_points callback gets called inside mock_network_receive
  constexpr cepton_sdk::SensorHandle dummy_handle = 0U;
  const cepton_sdk::SensorError ret =
    cepton_sdk::mock_network_receive(dummy_handle, timestamp_us, &pkt.data[0U], CEPTON_PACKET_SIZE);
  if (static_cast<int32_t>(CEPTON_SUCCESS) != static_cast<cepton_sdk::SensorErrorCode>(ret)) {
    throw ret;
  }
  // mock_network_receive may call a callback to which output is bound, thus meaning output.width
  // may be modified
  //lint -e948 NOLINT see above
  const bool8_t ready = output.width != 0U;
  //lint -e774 NOLINT see above
  if (ready) {
    // Transform points
    auto it_x = sensor_msgs::PointCloud2Iterator<float32_t>(output, "x");
    auto it_y = sensor_msgs::PointCloud2Iterator<float32_t>(output, "y");
    auto it_z = sensor_msgs::PointCloud2Iterator<float32_t>(output, "z");
    while ((it_x != it_x.end()) && (it_y != it_y.end()) && (it_z != it_z.end())) {
      m_transform.apply(*it_x, *it_y, *it_z);
      ++it_x;
      ++it_y;
      ++it_z;
    }
  }
  return ready;
}

////////////////////////////////////////////////////////////////////////////////
bool8_t CeptonCloudNode::get_output_remainder(sensor_msgs::msg::PointCloud2 & output)
{
  // This function is intended for when size(Packet) > size(Output), which should not be the case
  // here
  (void)output;
  return false;
}
////////////////////////////////////////////////////////////////////////////////
void CeptonCloudNode::check_fields(const sensor_msgs::msg::PointCloud2 & pcl) const
{
  // I know that we have four fields, in order: x-y-z-intensity
  if (4U != pcl.fields.size()) {
    throw std::runtime_error("Cepton cloud node: point cloud has unexpected number of fields");
  }
  const auto check_field = [](const sensor_msgs::msg::PointField & field) {
      if (sensor_msgs::msg::PointField::FLOAT32 != field.datatype) {
        throw std::runtime_error("PointCloud field is not of the right size");
      }
      if (0U != (field.offset % sizeof(float32_t))) {
        throw std::runtime_error("PointCloud field's offset is incorrect");
      }
    };
  check_field(pcl.fields[0U]);
  check_field(pcl.fields[1U]);
  check_field(pcl.fields[2U]);
  check_field(pcl.fields[3U]);
}
}  // namespace cepton_node
}  // namespace drivers
}  // namespace apex_auto
