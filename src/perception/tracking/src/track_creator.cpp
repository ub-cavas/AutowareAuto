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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wuseless-cast"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#pragma GCC diagnostic pop

#include <time_utils/time_utils.hpp>
#include <tracking/track_creator.hpp>

#include <functional>
#include <memory>
#include <set>
#include <vector>

namespace
{
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
}  // namespace

namespace autoware
{
namespace perception
{
namespace tracking
{

LidarOnlyPolicy::LidarOnlyPolicy(
  const float64_t default_variance,
  const float64_t noise_variance,
  const tf2::BufferCore & tf_buffer)
: m_default_variance{default_variance}, m_noise_variance{noise_variance}, m_tf_buffer{tf_buffer} {}

TrackCreationResult LidarOnlyPolicy::create(
  const ObjectsWithAssociations & objects) const
{
  TrackCreationResult retval;
  retval.associations = objects.associations();
  for (auto i = 0U; i < retval.associations.size(); ++i) {
    const auto & object = objects.objects().objects[i];
    auto & association = retval.associations[i];
    if (association.matched == Matched::kNothing) {
      // This object was not associated with anything before. It is now.
      retval.tracks.emplace_back(object, m_default_variance, m_noise_variance);
      const auto created_track_index = retval.tracks.size() - 1UL;
      association = {Matched::kNewTrack, created_track_index};
    }
  }
  return retval;
}

constexpr uint32_t LidarClusterIfVisionPolicy::kVisionCacheSize;

LidarClusterIfVisionPolicy::LidarClusterIfVisionPolicy(
  const VisionPolicyConfig & cfg,
  const float64_t default_variance,
  const float64_t noise_variance,
  const tf2::BufferCore & tf_buffer)
: m_default_variance{default_variance},
  m_noise_variance{noise_variance},
  m_tf_buffer{tf_buffer},
  m_cfg{cfg},
  m_associator{cfg.associator_cfg, tf_buffer} {}

void LidarClusterIfVisionPolicy::add_objects(
  const autoware_auto_perception_msgs::msg::ClassifiedRoiArray & vision_rois,
  const AssociatorResult & associator_result)
{
  autoware_auto_perception_msgs::msg::ClassifiedRoiArray::SharedPtr vision_rois_msg =
    std::make_shared<autoware_auto_perception_msgs::msg::ClassifiedRoiArray>();
  vision_rois_msg->header = vision_rois.header;
  for (const auto & unassigned_idx : associator_result.unassigned_detection_indices) {
    vision_rois_msg->rois.push_back(vision_rois.rois[unassigned_idx]);
  }

  const auto & frame_id = vision_rois_msg->header.frame_id;
  auto matching_vision_cache = m_vision_cache_map.find(frame_id);
  if (matching_vision_cache == m_vision_cache_map.end()) {
    const auto emplace_status = m_vision_cache_map.emplace(frame_id, kVisionCacheSize);
    matching_vision_cache = emplace_status.first;
  }
  matching_vision_cache->second.add(vision_rois_msg);
}

void LidarClusterIfVisionPolicy::create_using_cache(
  const ObjectsWithAssociations & objects,
  const VisionCache & vision_cache,
  TrackCreationResult & creator_ret) const
{
  // For foxy time has to be initialized explicitly with sec, nanosec constructor to use the
  // correct clock source when querying message_filters::cache.
  // Refer: https://github.com/ros2/message_filters/issues/32
  const rclcpp::Time t{objects.objects().header.stamp.sec, objects.objects().header.stamp.nanosec};
  const auto before = t - m_cfg.max_vision_lidar_timestamp_diff;
  const auto after = t + m_cfg.max_vision_lidar_timestamp_diff;
  const auto vision_msg_matches = vision_cache.getInterval(before, after);

  if (vision_msg_matches.empty()) {
    std::cerr << "No matching vision msgs for creating tracks" << std::endl;
    return;
  }

  const auto & vision_msg = *vision_msg_matches.back();
  const auto association_result = m_associator.assign(vision_msg, objects.objects());

  // This is not entirely correct as the images from different cameras might have different
  // timestamps but we assume they will be close enough.
  creator_ret.related_rois_stamp = vision_msg.header.stamp;

  for (auto cluster_idx = 0U; cluster_idx < objects.associations().size(); ++cluster_idx) {
    if (creator_ret.associations[cluster_idx].matched != Matched::kNothing) {continue;}
    if (association_result.track_assignments[cluster_idx] != AssociatorResult::UNASSIGNED) {
      creator_ret.tracks.emplace_back(
        objects.objects().objects[cluster_idx],
        vision_msg.rois[association_result.track_assignments[cluster_idx]].classifications,
        m_default_variance,
        m_noise_variance);
      const auto created_track_index = creator_ret.tracks.size() - 1UL;
      creator_ret.associations[cluster_idx] = {Matched::kNewTrack, created_track_index};
    }
  }
}

TrackCreationResult LidarClusterIfVisionPolicy::create(
  const ObjectsWithAssociations & objects) const
{
  TrackCreationResult retval;
  retval.associations = objects.associations();
  for (const auto & frame_cache : m_vision_cache_map) {
    create_using_cache(objects, frame_cache.second, retval);
  }
  return retval;
}

}  // namespace tracking
}  // namespace perception
}  // namespace autoware
