/// \copyright Copyright 2018 Apex.AI, Inc.
/// All rights reserved.
/// \file
/// \brief This file defines the cepton_node class.
#include <cepton_sdk_util.hpp>
#include <cstring>
#include <algorithm>
#include <utility>
#include "cepton_node/cepton_common.hpp"

namespace apex_auto
{
namespace drivers
{
namespace cepton_node
{

using cepton_sdk::util::SensorPoint;

////////////////////////////////////////////////////////////////////////////////
void block_callback(
  const cepton_sdk::SensorHandle handle,
  size64_t n_points,
  const cepton_sdk::SensorImagePoint * const c_points,
  void * const user_data)
{
  (void)handle;
  // user_data is m_points
  if (nullptr == user_data) {
    throw std::runtime_error("User data is null for some reason in cepton driver");
  }
  //lint -e925 -e{9079} Necessary to match external library's callback API NOLINT
  const auto points_ptr = static_cast<apex_auto_msgs::msg::PointBlock * const>(user_data);
  points_ptr->points.clear();
  // prevent overflow
  n_points = std::min(n_points, static_cast<size64_t>(apex_auto_msgs::msg::PointBlock::CAPACITY));
  // dump points from their format into our format
  for (size64_t idx = 0U; idx < n_points; ++idx) {
    SensorPoint pt;
    cepton_sdk::util::convert_sensor_image_point_to_point(c_points[idx], pt);
    apex_auto_msgs::msg::PointXYZIF msg_pt;
    // +x is forward for ROS, cepton uses +y forward
    msg_pt.x = pt.y;
    msg_pt.y = -pt.x;
    msg_pt.z = pt.z;
    msg_pt.intensity = pt.intensity;
    points_ptr->points.push_back(msg_pt);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Below disabled pclint expressions are related to conversions to unrelated pointers, and pointer
// arithmetic. In this case we disable these warnings because they are necessary to interface with
// an externally defined library, which uses a byte representation as its API.
//lint -e{9176, 9110, 9113, 2662, 9016} NOLINT
void cloud_callback(
  const cepton_sdk::SensorHandle handle,
  const size64_t n_points,
  const cepton_sdk::SensorImagePoint * const c_points,
  void * const user_data)
{
  (void)handle;
  // This check makes sure that when we do a byte-wise insert of their point struct, we won't go
  // past the end of the struct
  static_assert(sizeof(SensorPoint) >= ((4U * sizeof(float32_t)) + sizeof(int64_t)),
    "Cepton SensorPoint is unexpectedly small");
  // user_data is m_points
  if (nullptr == user_data) {
    throw std::runtime_error("User data is null for some reason in cepton driver");
  }
  //lint -e925 -e{9079} Necessary to match external library's callback API NOLINT
  const auto points_ptr = static_cast<sensor_msgs::msg::PointCloud2 * const>(user_data);
  // You should allocate a larger buffer
  if (n_points * 16U > points_ptr->data.capacity()) {
    throw std::runtime_error("Cepton PointCloud2 should be preallocated with more points");
  }
  // dump points from their format into our format
  for (size64_t idx = 0U; idx < n_points; ++idx) {
    if (c_points[idx].distance > 0.0F) {
      SensorPoint pt;
      cepton_sdk::util::convert_sensor_image_point_to_point(c_points[idx], pt);
      // random constant to cap intensity for better visualization
      constexpr float32_t max_intensity = 25.0F;
      pt.intensity = std::min(max_intensity, pt.intensity);
      // +x is forward for ROS, cepton uses +y forward
      std::swap(pt.x, pt.y);
      pt.y = -pt.y;
      // This works because I know the order is x-y-z, intensity might be somewhere else in the
      // struct, (e.g. v1.8 vs v1.9)
      auto old_last = &*points_ptr->data.end();
      constexpr auto size_xyz = sizeof(pt.x) + sizeof(pt.y) + sizeof(pt.z);
      points_ptr->data.resize(points_ptr->data.size() + size_xyz);
      (void)memmove(
        static_cast<void *>(old_last),
        static_cast<void *>(&pt.x),
        size_xyz);
      // Add intensity field, not contiguous in memory with x-y-z as of cepton_sdk v1.9
      old_last = &*points_ptr->data.end();
      constexpr auto size_intensity = sizeof(pt.intensity);
      points_ptr->data.resize(points_ptr->data.size() + size_intensity);
      (void)memmove(
        static_cast<void *>(old_last),
        static_cast<void *>(&pt.intensity),
        size_intensity);
      // Ignoring return value because I don't need a pointer to the first value
      ++points_ptr->width;
    }
  }
}
}  // namespace cepton_node
}  // namespace drivers
}  // namespace apex_auto
