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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef POINT_CLOUD_MAPPING__POINT_CLOUD_MAPPER_HPP_
#define POINT_CLOUD_MAPPING__POINT_CLOUD_MAPPER_HPP_

#include <point_cloud_mapping/visibility_control.hpp>
#include <localization_common/localizer_base.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <point_cloud_mapping/map.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <lidar_utils/point_cloud_utils.hpp>
#include <point_cloud_mapping/policies.hpp>
#include <helper_functions/message_adapters.hpp>
#include <time_utils/time_utils.hpp>
#include <experimental/optional>
#include <memory>
#include <string>
#include <utility>
#include <type_traits>

namespace autoware
{
namespace mapping
{
namespace point_cloud_mapping
{

template<typename MapperT>
class POINT_CLOUD_MAPPING_PUBLIC MapperBase : public common::helper_functions::crtp<MapperT>
{
public:
  template<typename MapIncrementT, typename PoseT>
  auto update_map(const MapIncrementT & msg, const PoseT & pose)
  {
    return this->impl().update_msp(msg, pose);
  }

  const typename MapperT::MapT & map() const noexcept
  {
    return this->impl().map();
  }

  const std::string & map_frame_id() const noexcept
  {
    return this->impl().map_frame_id();
  }

  const std::chrono::system_clock::time_point & map_time_stamp() const noexcept
  {
    return this->impl().map_time_stamp();
  }
};


/// Virtual base class of the mapper.
template<typename MapRepresentationT, typename MapIncrementT>
class POINT_CLOUD_MAPPING_PUBLIC PointCloudMapper
  : public MapperBase<PointCloudMapper<MapRepresentationT, MapIncrementT>>
{
public:
  /// Constructor.
  explicit PointCloudMapper(MapRepresentationT && map, const std::string & map_frame_id)
  : m_map{std::forward<MapRepresentationT>(map)}, m_map_frame_id{map_frame_id} {}

  /// Pass the increment to the map. The default behavior is to expect that the map
  /// and the localizer have independent representations and push the increment to both
  /// the underlying map representation and the localizer.
  /// \param increment Increment to pass to the maps.
  /// \param pose Pose to be inserted with the increment.
  auto update_map(
    const MapIncrementT & increment,
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
  {
    using common::helper_functions::message_field_adapters::get_frame_id;
    const auto & msg_frame = get_frame_id(increment);
    if (increment != m_map_frame_id) {
      throw std::runtime_error("MapperBase: Can't insert a map that is in a different frame.");
    }
    const auto insert_summary = m_map.try_add_observation(increment, pose);

    // Let's update the stamp at the start of each map in case someone needs it.
    if (insert_summary.update_type == MapUpdateType::NEW) {
      m_current_stamp = time_utils::from_message(pose.header.stamp);
    }
    return insert_summary;
  }

  const MapRepresentationT & map() const noexcept {return m_map;}

  const std::string & map_frame_id() const noexcept
  {
    return m_map_frame_id;
  }

  std::chrono::system_clock::time_point map_time_stamp() const noexcept
  {
    return m_current_stamp;
  }


  MapRepresentationT m_map;
  std::string m_map_frame_id;
  // TODO(igor): is this a good name? What is this variable for?
  std::chrono::system_clock::time_point m_current_stamp{
    std::chrono::system_clock::time_point::min()};
};

}  // namespace point_cloud_mapping
}  // namespace mapping
}  // namespace autoware

#endif  // POINT_CLOUD_MAPPING__POINT_CLOUD_MAPPER_HPP_
