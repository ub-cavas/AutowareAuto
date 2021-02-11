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

#include "lanelet2_map_provider/lanelet2_map_provider.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <common/types.hpp>
#include <string>

#include "had_map_utils/had_map_utils.hpp"

using autoware::common::types::float64_t;

namespace autoware
{

namespace lanelet2_map_provider
{

Lanelet2MapProvider::Lanelet2MapProvider(
  const std::string & map_filename,
  float64_t x_origin_offset,
  float64_t y_origin_offset,
  float64_t z_origin_offset)
: m_map_filename{map_filename},
  m_x_origin_offset{x_origin_offset},
  m_y_origin_offset{y_origin_offset},
  m_z_origin_offset{z_origin_offset}
{
  earth = GeographicLib::Geocentric(
    GeographicLib::Constants::WGS84_a(),
    GeographicLib::Constants::WGS84_f());
}

void Lanelet2MapProvider::set_earth_to_map_transform(
  const geometry_msgs::msg::TransformStamped & stf)
{
  // Need to convert earth to map transform back to lat lon for lanelet2 projector
  earth.Reverse(
    stf.transform.translation.x + m_x_origin_offset,
    stf.transform.translation.y + m_y_origin_offset,
    stf.transform.translation.z + m_z_origin_offset,
    m_origin_lat, m_origin_lon, m_origin_ele);
}

void Lanelet2MapProvider::set_geographic_coords(
  const float64_t lat, const float64_t lon, const float64_t ele)
{
  // Need to convert from lat/lon to XYZ then back again to apply offsets
  float64_t x{0.0}, y{0.0}, z{0.0};

  earth.Forward(lat, lon, ele, x, y, z);
  earth.Reverse(
    x + m_x_origin_offset,
    y + m_y_origin_offset,
    z + m_z_origin_offset,
    m_origin_lat, m_origin_lon, m_origin_ele);
}

void Lanelet2MapProvider::load_map()
{
  lanelet::ErrorMessages errors;
  lanelet::GPSPoint originGps{m_origin_lat, m_origin_lon, m_origin_ele};
  lanelet::Origin origin{originGps};

  lanelet::projection::UtmProjector projector(origin);
  m_map = lanelet::load(m_map_filename, projector, &errors);
  autoware::common::had_map_utils::overwriteLaneletsCenterline(m_map, true);
}

}  // namespace lanelet2_map_provider

}  // namespace autoware
