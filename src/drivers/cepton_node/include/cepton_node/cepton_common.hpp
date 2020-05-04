/// \copyright Copyright 2018 Apex.AI, Inc.
/// All rights reserved.
/// \file
/// \brief This file defines the cepton_node class.

#ifndef CEPTON_NODE__CEPTON_COMMON_HPP_
#define CEPTON_NODE__CEPTON_COMMON_HPP_

#include <cepton_node/visibility_control.hpp>
#include <apex_auto_msgs/msg/point_block.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cepton_sdk.hpp>

namespace apex_auto
{
namespace drivers
{
/// \brief A package which encapsulates the Cepton LiDAR drivers in ROS 2
namespace cepton_node
{
namespace
{
/// \brief Internal init function common to specializations of cepton_init
/// \param[in] options SDK setup options
/// \throw std::runtime_error If initialization fails or sensor_idx is outside of bounds
CEPTON_NODE_LOCAL void cepton_init_internal(const cepton_sdk::Options & options)
{
  cepton_sdk::SensorError ret;
  ret = cepton_sdk::initialize(CEPTON_SDK_VERSION, options, NULL, NULL);
  bool8_t not_ok =
    static_cast<int32_t>(CEPTON_SUCCESS) != static_cast<cepton_sdk::SensorErrorCode>(ret);
  not_ok = (static_cast<int32_t>(CEPTON_ERROR_ALREADY_INITIALIZED) !=
    static_cast<cepton_sdk::SensorErrorCode>(ret)) && not_ok;
  if (not_ok) {
    throw ret;
  }
}
}  // namespace

/// \brief This magic number comes from the Cepton networking example
constexpr uint32_t CEPTON_PACKET_SIZE = 1360U;  // 1318U;
/// \brief A wrapper around a plain buffer for receiving a UDP packet
struct CEPTON_NODE_LOCAL Packet
{
  uint8_t data[CEPTON_PACKET_SIZE];
  static_assert(
    static_cast<uint32_t>(apex_auto_msgs::msg::PointBlock::CAPACITY) >=
    CEPTON_SDK_MAX_POINTS_PER_PACKET,
    "Can not fit whole cepton packet into point block");
};

/// \brief Globally visible callback function which deserialize Cepton points into a common
///        PointBlock representation
/// \param[in] handle Identifier for the sensor for whom points are being handled
/// \param[in] n_points Number of points in the point buffer
/// \param[in] c_points Point buffer from the Cepton SDK
/// \param[out] user_data Points to a PointBlock used for intermediate storage of point data
CEPTON_NODE_LOCAL void block_callback(
  const cepton_sdk::SensorHandle handle,
  size64_t n_points,
  const cepton_sdk::SensorImagePoint * const c_points,
  void * const user_data);

/// \brief Globally visible callback function which deserialize Cepton points into a common
///        PointCloud2 representation
/// \param[in] handle Identifier for the sensor for whom points are being handled
/// \param[in] n_points Number of points in the point buffer
/// \param[in] c_points Point buffer from the Cepton SDK
/// \param[out] user_data Points to a PointCloud2 used for intermediate storage of point data
CEPTON_NODE_LOCAL void cloud_callback(
  const cepton_sdk::SensorHandle handle,
  const size64_t n_points,
  const cepton_sdk::SensorImagePoint * const c_points,
  void * const user_data);

/// \brief Initialize the Cepton SDK
/// \tparam OutputT The type of user_data, currently only specialized for PointCloud2 and PointBlock
///                 messages
/// \param[inout] user_data A reference to the object that will store points for intermediate usage
/// \throw std::runtime_error If any part of initialization failed
template<typename OutputT>
void cepton_init(OutputT & user_data);

/// \brief Initialize the Cepton SDK with a poitn callback for PointBlock
/// \param[inout] user_data A reference to the PointBlock that will store points for intermediate
///                         usage
/// \throw std::runtime_error If any part of initialization failed
template<>
CEPTON_NODE_LOCAL inline void cepton_init(apex_auto_msgs::msg::PointBlock & user_data)
{
  // initialize cepton sdk
  cepton_sdk::Options options = cepton_sdk_create_options();
  options.frame.mode = static_cast<uint32_t>(CEPTON_SDK_FRAME_STREAMING);
  options.control_flags |= static_cast<uint32_t>(CEPTON_SDK_CONTROL_DISABLE_NETWORK);
  cepton_init_internal(options);
  // register callback
  user_data.points.reserve(apex_auto_msgs::msg::PointBlock::CAPACITY);
  //lint -e{941} Necessary to interface with external library NOLINT
  void * const user_data_ptr = static_cast<void *>(&user_data);
  const cepton_sdk::SensorError ret =
    cepton_sdk::listen_image_frames(&block_callback, user_data_ptr);
  if (static_cast<int32_t>(CEPTON_SUCCESS) != static_cast<cepton_sdk::SensorErrorCode>(ret)) {
    throw ret;
  }
}

/// \brief Initialize the Cepton SDK with a point callback for PointCloud2
/// \param[inout] user_data A reference to the PointCloud2 that will store points for intermediate
///                         usage
/// \throw std::runtime_error If any part of initialization failed
template<>
CEPTON_NODE_LOCAL inline void cepton_init(sensor_msgs::msg::PointCloud2 & user_data)
{
  // initialize cepton sdk
  cepton_sdk::Options options = cepton_sdk::create_options();
  options.frame.mode = static_cast<uint32_t>(CEPTON_SDK_FRAME_COVER);
  options.control_flags |= static_cast<uint32_t>(CEPTON_SDK_CONTROL_DISABLE_NETWORK);
  cepton_init_internal(options);
  // register callback
  //lint -e941 Necessary to interface with external library NOLINT
  void * const user_data_ptr = static_cast<void *>(&user_data);
  const cepton_sdk::SensorError ret =
    cepton_sdk::listen_image_frames(&cloud_callback, user_data_ptr);
  if (static_cast<int32_t>(CEPTON_SUCCESS) != static_cast<cepton_sdk::SensorErrorCode>(ret)) {
    throw ret;
  }
}
}  // namespace cepton_node
}  // namespace drivers
}  // namespace apex_auto


#endif  // CEPTON_NODE__CEPTON_COMMON_HPP_
