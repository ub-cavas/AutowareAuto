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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef TRACKING__OBJECTS_WITH_ASSOCIATIONS_HPP_
#define TRACKING__OBJECTS_WITH_ASSOCIATIONS_HPP_


#include <autoware_auto_msgs/msg/classified_roi_array.hpp>
#include <autoware_auto_msgs/msg/detected_objects.hpp>
#include <autoware_auto_msgs/msg/tracked_objects.hpp>
#include <tracking/visibility_control.hpp>

#include <utility>
#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{

namespace detail
{
template<class MsgT>
std::size_t get_size(const MsgT & msg);

template<>
inline std::size_t get_size(const autoware_auto_msgs::msg::DetectedObjects & msg)
{
  return msg.objects.size();
}
}  // namespace detail

///
/// @brief      This enum holds various values that signify a match to a certain modality.
///
enum class Matched
{
  kNothing,  ///< Nothing was matched.
  kOtherDetection,  ///< Matched another detection.
  kExistingTrack,  ///< Matched an existing track.
  kNewTrack  ///< Matched a new track.
};

///
/// @brief      A struct that represents an association. It holds an enum that indicates the type of
///             a match and an index of an object to which this association is matched.
///
struct Association
{
  Matched matched;
  std::size_t match_index;
};

using Associations = std::vector<Association>;

///
/// @brief      This class describes an associated object array of a certain message type.
///
/// @details    It assumes that the MsgT has a number of messages in it that all must be associated
///             with some other instances.
///
/// @tparam     MsgT  Type of message that is associated with some other instances.
///
template<class MsgT>
class TRACKING_PUBLIC Associated
{
public:
  explicit Associated(const MsgT & objects, const Associations & associations)
  : m_objects{objects}, m_associations{associations}
  {
    if (detail::get_size(m_objects) != m_associations.size()) {
      throw std::runtime_error("Objects number must match the associations number");
    }
  }

  explicit Associated(const MsgT & objects)
  : m_objects{objects}, m_associations(detail::get_size(m_objects), {Matched::kNothing, 0UL}) {}

  explicit Associated(MsgT && objects)
  : m_objects{std::move(objects)},
    m_associations(detail::get_size(m_objects), {Matched::kNothing, 0UL}) {}

  const MsgT & objects() const noexcept {return m_objects;}

  const Associations & associations() const noexcept
  {
    return m_associations;
  }
  Associations & associations() noexcept {return m_associations;}

private:
  MsgT m_objects;
  Associations m_associations;
};

using ObjectsWithAssociations = Associated<autoware_auto_msgs::msg::DetectedObjects>;
using RoisWithAssociations = Associated<autoware_auto_msgs::msg::ClassifiedRoiArray>;
using TracksWithAssociations = Associated<autoware_auto_msgs::msg::ClassifiedRoiArray>;

}  // namespace tracking
}  // namespace perception
}  // namespace autoware


#endif  // TRACKING__OBJECTS_WITH_ASSOCIATIONS_HPP_
