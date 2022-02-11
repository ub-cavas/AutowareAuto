// Copyright 2018 the Autoware Foundation
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
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.


/// \copyright Copyright 2017-2018 the Autoware Foundation
/// All rights reserved.
/// \file
/// \brief This file defines a simple ROS 2 velodyne driver that publishes full point clouds

#ifndef VELODYNE_NODES__VELODYNE_CLOUD_NODE_HPP_
#define VELODYNE_NODES__VELODYNE_CLOUD_NODE_HPP_

#include <string>
#include <vector>
#include "common/types.hpp"
#include "lidar_utils/point_cloud_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "udp_driver/udp_driver.hpp"
#include "velodyne_driver/velodyne_translator.hpp"
#include "velodyne_nodes/visibility_control.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using autoware::common::types::bool8_t;

namespace autoware
{
namespace drivers
{
/// \brief Resources for nodes that use the `velodyne_driver`
namespace velodyne_nodes
{

template<typename CloudModifierT>
struct add_point_to_cloud_modifier;

template<>
struct add_point_to_cloud_modifier<autoware::common::lidar_utils::CloudModifier>
{
  add_point_to_cloud_modifier(
    autoware::common::lidar_utils::CloudModifier & modifier,
    const autoware::common::types::PointXYZIF & pt)
  {
    using autoware::common::types::PointXYZI;
    modifier.push_back(PointXYZI{pt.x, pt.y, pt.z, pt.intensity});
  }
};

template<>
struct add_point_to_cloud_modifier<autoware::common::lidar_utils::CloudModifierRing>
{
  add_point_to_cloud_modifier(
    autoware::common::lidar_utils::CloudModifierRing & modifier,
    const autoware::common::types::PointXYZIF & pt)
  {
    modifier.push_back(pt);
  }
};

/// Template class for the velodyne driver node that receives veldyne `packet`s via
/// UDP, converts the packet into a PointCloud2 message and publishes this cloud.
/// \tparam SensorData SensorData implementation for the specific velodyne sensor model.
template<typename SensorData>
class VELODYNE_NODES_PUBLIC VelodyneCloudNode : public rclcpp::Node
{
public:
  using VelodyneTranslatorT = velodyne_driver::VelodyneTranslator<SensorData>;
  using Config = typename VelodyneTranslatorT::Config;
  using Packet = typename VelodyneTranslatorT::Packet;

  VelodyneCloudNode(const std::string & node_name, const rclcpp::NodeOptions & options);

  /// Handle data packet from the udp driver
  /// \param buffer Data from the udp driver
  void receiver_callback(const std::vector<uint8_t> & buffer);

protected:
  void init_output(sensor_msgs::msg::PointCloud2 & output);
  bool8_t convert(
    const Packet & pkt,
    sensor_msgs::msg::PointCloud2 & output);

  template<typename CloudModifierT>
  bool8_t convert_impl(
    const Packet & pkt,
    sensor_msgs::msg::PointCloud2 & output)
  {
    // This handles the case when the below loop exited due to containing extra points
    using autoware::common::types::PointXYZIF;
    CloudModifierT modifier{output};
    if (m_published_cloud) {
      // reset the pointcloud
      modifier.clear();
      modifier.reserve(m_cloud_size);
      m_point_cloud_idx = 0;

      // deserialize remainder into pointcloud
      m_published_cloud = false;
      for (uint32_t idx = m_remainder_start_idx; idx < m_point_block.size(); ++idx) {
        const autoware::common::types::PointXYZIF & pt = m_point_block[idx];
        add_point_to_cloud_modifier<CloudModifierT>(modifier, pt);
        m_point_cloud_idx++;
      }
    }
    m_translator.convert(pkt, m_point_block);
    for (uint32_t idx = 0U; idx < m_point_block.size(); ++idx) {
      const autoware::common::types::PointXYZIF & pt = m_point_block[idx];
      if (static_cast<uint16_t>(autoware::common::types::PointXYZIF::END_OF_SCAN_ID) != pt.id) {
        add_point_to_cloud_modifier<CloudModifierT>(modifier, pt);
        m_point_cloud_idx++;
        if (modifier.size() >= m_cloud_size) {
          m_published_cloud = true;
          m_remainder_start_idx = idx;
        }
      } else {
        m_published_cloud = true;
        m_remainder_start_idx = idx;
        break;
      }
    }
    if (m_published_cloud) {
      // resize pointcloud down to its actual size
      modifier.resize(m_point_cloud_idx);
      output.header.stamp = this->now();
    }

    return m_published_cloud;
  }

  bool8_t get_output_remainder(sensor_msgs::msg::PointCloud2 & output);

private:
  void init_udp_driver();

  IoContext m_io_cxt;
  ::drivers::udp_driver::UdpDriver m_udp_driver;
  VelodyneTranslatorT m_translator;
  std::vector<autoware::common::types::PointXYZIF> m_point_block;

  std::string m_ip;
  uint16_t m_port;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pc2_pub_ptr;
  sensor_msgs::msg::PointCloud2 m_pc2_msg{};
  bool m_published_cloud = false;
  // Keeps track of where you left off on the converted point block in case you needed to publish
  // a point cloud in the middle of processing it
  uint32_t m_remainder_start_idx;
  // keeps track of the constructed point cloud to continue growing it with new data
  uint32_t m_point_cloud_idx;
  const std::string m_frame_id;
  const std::uint32_t m_cloud_size;
  bool8_t m_ring_information;
};  // class VelodyneCloudNode

class VELODYNE_NODES_PUBLIC VelodyneCloudWrapperNode : public rclcpp::Node
{
private:
  VelodyneCloudNode<velodyne_driver::VLP16Data>::SharedPtr vlp16_driver_node_ptr_ = nullptr;
  VelodyneCloudNode<velodyne_driver::VLP32CData>::SharedPtr vlp32c_driver_node_ptr_ = nullptr;
  VelodyneCloudNode<velodyne_driver::VLS128Data>::SharedPtr vls128_driver_node_ptr_ = nullptr;

public:
  explicit VelodyneCloudWrapperNode(const rclcpp::NodeOptions & node_options);
};

class VELODYNE_NODES_PUBLIC VLP16DriverNode : public VelodyneCloudNode<velodyne_driver::VLP16Data>
{
private:
  VelodyneCloudNode<velodyne_driver::VLP16Data>::SharedPtr vlp16_driver_node_ptr_;

public:
  explicit VLP16DriverNode(const rclcpp::NodeOptions & node_options);
};

class VELODYNE_NODES_PUBLIC VLP32CDriverNode : public VelodyneCloudNode<velodyne_driver::VLP32CData>
{
public:
  explicit VLP32CDriverNode(const rclcpp::NodeOptions & node_options);
};

class VELODYNE_NODES_PUBLIC VLS128DriverNode : public VelodyneCloudNode<velodyne_driver::VLS128Data>
{
public:
  explicit VLS128DriverNode(const rclcpp::NodeOptions & node_options);
};

}  // namespace velodyne_nodes
}  // namespace drivers
}  // namespace autoware

#endif  // VELODYNE_NODES__VELODYNE_CLOUD_NODE_HPP_
