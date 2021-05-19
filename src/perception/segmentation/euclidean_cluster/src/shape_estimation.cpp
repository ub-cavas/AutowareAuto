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

#include <euclidean_cluster/shape_estimation.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include <geometry/bounding_box_2d.hpp>

namespace autoware
{
namespace perception
{
namespace segmentation
{
namespace euclidean_cluster
{

////////////////////////////////////////////////////////////////////////////////
ComputeLfitBoundingBoxes::ComputeLfitBoundingBoxes(
  const autoware_auto_msgs::msg::PointClusters & clusters)
: m_clusters(clusters) {}

void ComputeLfitBoundingBoxes::operator()(
  autoware_auto_msgs::msg::DetectedObjects & objects_msg)
{
  objects_msg.objects.clear();
  for (uint32_t cls_id = 0U; cls_id < m_clusters.cluster_boundary.size(); cls_id++) {
    try {
      const auto iter_pair = common::lidar_utils::get_cluster(m_clusters, cls_id);
      if (iter_pair.first == iter_pair.second) {
        continue;
      }
      objects_msg.objects.push_back(
        common::geometry::bounding_box::details::get_detected_object(
          common::geometry::bounding_box::lfit_bounding_box_2d(
            iter_pair.first, iter_pair.second)));
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
    }
  }
}

void ComputeLfitBoundingBoxes::operator()(autoware_auto_msgs::msg::BoundingBoxArray & boxes)
{
  boxes.boxes.clear();
  for (uint32_t cls_id = 0U; cls_id < m_clusters.cluster_boundary.size(); cls_id++) {
    try {
      const auto iter_pair = common::lidar_utils::get_cluster(m_clusters, cls_id);
      if (iter_pair.first == iter_pair.second) {
        continue;
      }
      boxes.boxes.push_back(
        common::geometry::bounding_box::lfit_bounding_box_2d(iter_pair.first, iter_pair.second));
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
ComputeLfitBoundingBoxesWithZ::ComputeLfitBoundingBoxesWithZ(
  const autoware_auto_msgs::msg::PointClusters & clusters)
: m_clusters(clusters) {}

void ComputeLfitBoundingBoxesWithZ::operator()(
  autoware_auto_msgs::msg::DetectedObjects & objects_msg)
{
  objects_msg.objects.clear();
  for (uint32_t cls_id = 0U; cls_id < m_clusters.cluster_boundary.size(); cls_id++) {
    try {
      const auto iter_pair = common::lidar_utils::get_cluster(m_clusters, cls_id);
      if (iter_pair.first == iter_pair.second) {
        continue;
      }
      objects_msg.objects.push_back(
        common::geometry::bounding_box::details::get_detected_object(
          common::geometry::bounding_box::lfit_bounding_box_2d(
            iter_pair.first,
            iter_pair.second)));
      common::geometry::bounding_box::compute_height(
        iter_pair.first, iter_pair.second, objects_msg.objects.back().shape);
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
    }
  }
}

void ComputeLfitBoundingBoxesWithZ::operator()(autoware_auto_msgs::msg::BoundingBoxArray & boxes)
{
  boxes.boxes.clear();
  for (uint32_t cls_id = 0U; cls_id < m_clusters.cluster_boundary.size(); cls_id++) {
    try {
      const auto iter_pair = common::lidar_utils::get_cluster(m_clusters, cls_id);
      if (iter_pair.first == iter_pair.second) {
        continue;
      }
      boxes.boxes.push_back(
        common::geometry::bounding_box::lfit_bounding_box_2d(iter_pair.first, iter_pair.second));
      common::geometry::bounding_box::compute_height(
        iter_pair.first, iter_pair.second, boxes.boxes.back());
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
ComputeEigenBoxes::ComputeEigenBoxes(const autoware_auto_msgs::msg::PointClusters & clusters)
: m_clusters(clusters) {}

void ComputeEigenBoxes::operator()(autoware_auto_msgs::msg::DetectedObjects & objects_msg)
{
  objects_msg.objects.clear();
  for (uint32_t cls_id = 0U; cls_id < m_clusters.cluster_boundary.size(); cls_id++) {
    try {
      const auto iter_pair = common::lidar_utils::get_cluster(m_clusters, cls_id);
      if (iter_pair.first == iter_pair.second) {
        continue;
      }
      objects_msg.objects.push_back(
        common::geometry::bounding_box::details::get_detected_object(
          common::geometry::bounding_box::eigenbox_2d(iter_pair.first, iter_pair.second)));
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
    }
  }
}

void ComputeEigenBoxes::operator()(autoware_auto_msgs::msg::BoundingBoxArray & boxes)
{
  boxes.boxes.clear();
  for (uint32_t cls_id = 0U; cls_id < m_clusters.cluster_boundary.size(); cls_id++) {
    try {
      const auto iter_pair = common::lidar_utils::get_cluster(m_clusters, cls_id);
      if (iter_pair.first == iter_pair.second) {
        continue;
      }
      boxes.boxes.push_back(
        common::geometry::bounding_box::eigenbox_2d(iter_pair.first, iter_pair.second));
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
ComputeEigenBoxesWithZ::ComputeEigenBoxesWithZ(
  const autoware_auto_msgs::msg::PointClusters & clusters)
: m_clusters(clusters) {}

void ComputeEigenBoxesWithZ::operator()(
  autoware_auto_msgs::msg::DetectedObjects & objects_msg)
{
  objects_msg.objects.clear();
  for (uint32_t cls_id = 0U; cls_id < m_clusters.cluster_boundary.size(); cls_id++) {
    try {
      const auto iter_pair = common::lidar_utils::get_cluster(m_clusters, cls_id);
      if (iter_pair.first == iter_pair.second) {
        continue;
      }
      objects_msg.objects.push_back(
        common::geometry::bounding_box::details::get_detected_object(
          common::geometry::bounding_box::eigenbox_2d(iter_pair.first, iter_pair.second)));
      common::geometry::bounding_box::compute_height(
        iter_pair.first, iter_pair.second, objects_msg.objects.back().shape);
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
    }
  }
}

void ComputeEigenBoxesWithZ::operator()(
  autoware_auto_msgs::msg::BoundingBoxArray & boxes)
{
  boxes.boxes.clear();
  for (uint32_t cls_id = 0U; cls_id < m_clusters.cluster_boundary.size(); cls_id++) {
    try {
      const auto iter_pair = common::lidar_utils::get_cluster(m_clusters, cls_id);
      if (iter_pair.first == iter_pair.second) {
        continue;
      }
      boxes.boxes.push_back(
        common::geometry::bounding_box::eigenbox_2d(iter_pair.first, iter_pair.second));
      common::geometry::bounding_box::compute_height(
        iter_pair.first, iter_pair.second, boxes.boxes.back());
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
    }
  }
}
////////////////////////////////////////////////////////////////////////////////

}  // namespace euclidean_cluster
}  // namespace segmentation
}  // namespace perception
}  // namespace autoware
