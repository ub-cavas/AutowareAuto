// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef TRACKING__TRACK_CREATOR_HPP_
#define TRACKING__TRACK_CREATOR_HPP_

#include <autoware_auto_perception_msgs/msg/classified_roi_array.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <common/types.hpp>
#include <message_filters/cache.h>
#include <tf2/buffer_core.h>
#include <tracking/greedy_roi_associator.hpp>
#include <tracking/objects_with_associations.hpp>
#include <tracking/tracked_object.hpp>
#include <tracking/tracker_types.hpp>
#include <tracking/visibility_control.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{

/// Struct defining configuration parameters for LidarIfVision policy
struct TRACKING_PUBLIC VisionPolicyConfig
{
  // Struct defining parameters for vision association
  GreedyRoiAssociatorConfig associator_cfg;
  // Maximum allowed difference in the timestamp of the two messages being associated
  std::chrono::milliseconds max_vision_lidar_timestamp_diff{20};
};

/// Struct to be used as the return value from track creation process
struct TRACKING_PUBLIC TrackCreationResult
{
  /// List of newly created tracks
  std::vector<TrackedObject> tracks;
  /// List of associations that matches an association to each track
  Associations associations;
  /// Timestamps of msgs from each of the ClassifiedROIArray topics used for track creation
  builtin_interfaces::msg::Time related_rois_stamp;
};

// Class implementing LidarOnly track creation policy
class TRACKING_PUBLIC LidarOnlyPolicy
{
public:
  LidarOnlyPolicy(
    const common::types::float64_t default_variance,
    const common::types::float64_t noise_variance,
    const tf2::BufferCore & tf_buffer);

  TrackCreationResult create(const ObjectsWithAssociations & objects) const;

private:
  common::types::float64_t m_default_variance;
  common::types::float64_t m_noise_variance;
  const tf2::BufferCore & m_tf_buffer;
};

/// Class implementing LidarIfVision track creation policy
class TRACKING_PUBLIC LidarClusterIfVisionPolicy
{
public:
  LidarClusterIfVisionPolicy(
    const VisionPolicyConfig & cfg,
    const common::types::float64_t default_variance,
    const common::types::float64_t noise_variance,
    const tf2::BufferCore & tf_buffer);

  TrackCreationResult create(const ObjectsWithAssociations & objects) const;

  void add_objects(
    const autoware_auto_perception_msgs::msg::ClassifiedRoiArray & vision_rois,
    const AssociatorResult & associator_result);

private:
  using VisionCache =
    message_filters::Cache<autoware_auto_perception_msgs::msg::ClassifiedRoiArray>;

  void create_using_cache(
    const ObjectsWithAssociations & objects,
    const VisionCache & vision_cache,
    TrackCreationResult & result) const;

  static constexpr std::uint32_t kVisionCacheSize = 20U;

  common::types::float64_t m_default_variance;
  common::types::float64_t m_noise_variance;
  const tf2::BufferCore & m_tf_buffer;

  VisionPolicyConfig m_cfg;
  GreedyRoiAssociator m_associator;

  std::unordered_map<std::string, VisionCache> m_vision_cache_map;
};

/// \brief Class to create new tracks based on a predefined policy and unassociated detections
template<class PolicyT>
class TRACKING_PUBLIC TrackCreator
{
public:
  /// \brief Constructor
  explicit TrackCreator(std::shared_ptr<PolicyT> policy)
  : m_policy{policy} {}
  /// \brief Create new tracks based on the policy and unassociated detections. Call the
  ///        appropriate add_unassigned_* functions before calling this.
  /// \return vector of newly created TrackedObject objects
  inline TrackCreationResult create_tracks(const ObjectsWithAssociations & objects)
  {
    return m_policy->create(objects);
  }

  /// Function to add unassigned detections. This function just passes through the arguments.
  /// The actual implementation is handled by the individual policy implementation classes.
  /// Refer to the CreationPolicyBase doc for details
  template<typename ... Ts>
  inline auto add_objects(Ts && ... args)
  {
    return m_policy->add_objects(std::forward<Ts>(args)...);
  }

private:
  std::shared_ptr<PolicyT> m_policy{};
};

}  // namespace tracking
}  // namespace perception
}  // namespace autoware

#endif   // TRACKING__TRACK_CREATOR_HPP_
