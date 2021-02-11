// Copyright 2020 The Autoware Foundation
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

/// \copyright Copyright 2020 The Autoware Foundation
/// \file
/// \brief This file defines the lanelet2_map_provider class.

#ifndef LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_HPP_
#define LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_HPP_

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <common/types.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_map_provider/visibility_control.hpp>

#include <iostream>
#include <string>
#include <memory>

#include "autoware_auto_msgs/msg/had_map_bin.hpp"

using autoware::common::types::float64_t;

namespace autoware
{
/// \brief TODO(simon.thompson): Document namespaces!
namespace lanelet2_map_provider
{

/// \class Lanelet2MapProvider
/// \brief Provides functoins to load and access a lanelet2 OSM map.
class LANELET2_MAP_PROVIDER_PUBLIC Lanelet2MapProvider
{
public:
  /// \brief Default constructor
  /// \param[in] map_filename Absolute path to the OSM file on disk
  /// \param[in] x_origin_offset An offset to apply to the X value of the map origin in meters
  /// \param[in] y_origin_offset An offset to apply to the Y value of the map origin in meters
  /// \param[in] z_origin_offset An offset to apply to the Z value of the map origin in meters
  Lanelet2MapProvider(
    const std::string & map_filename,
    float64_t x_origin_offset,
    float64_t y_origin_offset,
    float64_t z_origin_offset);

  /// \brief set the transform between earth and map frames for projection of map data
  /// \param stf the earth to map transform
  void set_earth_to_map_transform(const geometry_msgs::msg::TransformStamped & stf);

  /// \brief directly set hte geographic coordinates of the map orgin
  /// \param lat map origin latitude
  /// \param lon map orgin longitude
  /// \param ele map orgin elevation
  void set_geographic_coords(const float64_t lat, const float64_t lon, const float64_t ele);

  /// \brief load the lanelet map and project into the coordinates of the origin
  void load_map();

  std::shared_ptr<lanelet::LaneletMap> m_map;

private:
  std::string m_map_filename;
  // map orgin as a transform from earth center
  geometry_msgs::msg::TransformStamped m_earth_to_map;
  float64_t m_origin_lat;  // map orgin in latitude, longitude and elevation
  float64_t m_origin_lon;
  float64_t m_origin_ele;
  float64_t m_x_origin_offset;  ///< X offset in meters to apply to map origin
  float64_t m_y_origin_offset;  ///< Y offset in meters to apply to map origin
  float64_t m_z_origin_offset;  ///< Z offset in meters to apply to map origin
  GeographicLib::Geocentric earth;
};

}  // namespace lanelet2_map_provider

}  // namespace autoware

#endif  // LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_HPP_
