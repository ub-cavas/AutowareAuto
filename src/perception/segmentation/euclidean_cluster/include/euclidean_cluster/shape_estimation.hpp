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

#ifndef EUCLIDEAN_CLUSTER__SHAPE_ESTIMATION_HPP_
#define EUCLIDEAN_CLUSTER__SHAPE_ESTIMATION_HPP_

#include <autoware_auto_msgs/msg/bounding_box_array.hpp>
#include <autoware_auto_msgs/msg/detected_objects.hpp>
#include <autoware_auto_msgs/msg/point_clusters.hpp>
#include <euclidean_cluster/visibility_control.hpp>

/// \brief This file contains functions that convert PointClusters to required shapes
namespace autoware
{
namespace perception
{
namespace segmentation
{
namespace euclidean_cluster
{
/// \brief Compute lfit bounding boxes for the given cluster and assign to type based on variant
///        used
struct EUCLIDEAN_CLUSTER_PUBLIC ComputeLfitBoundingBoxes
{
  explicit ComputeLfitBoundingBoxes(const autoware_auto_msgs::msg::PointClusters & clusters);
  void operator()(autoware_auto_msgs::msg::DetectedObjects & objects_msg);
  void operator()(autoware_auto_msgs::msg::BoundingBoxArray & boxes);

  autoware_auto_msgs::msg::PointClusters m_clusters;
};

/// \brief Compute lfit bounding boxes with z for the given cluster and assign to type based on
///        variant used
struct EUCLIDEAN_CLUSTER_PUBLIC ComputeLfitBoundingBoxesWithZ
{
  explicit ComputeLfitBoundingBoxesWithZ(const autoware_auto_msgs::msg::PointClusters & clusters);
  void operator()(autoware_auto_msgs::msg::DetectedObjects & objects_msg);
  void operator()(autoware_auto_msgs::msg::BoundingBoxArray & boxes);

  autoware_auto_msgs::msg::PointClusters m_clusters;
};

/// \brief Compute eigen bounding boxes for the given cluster and assign to type based on variant
///        used
struct EUCLIDEAN_CLUSTER_PUBLIC ComputeEigenBoxes
{
  explicit ComputeEigenBoxes(const autoware_auto_msgs::msg::PointClusters & clusters);
  void operator()(autoware_auto_msgs::msg::DetectedObjects & objects_msg);
  void operator()(autoware_auto_msgs::msg::BoundingBoxArray & boxes);

  const autoware_auto_msgs::msg::PointClusters & m_clusters;
};

/// \brief Compute eigen bounding boxes with z for the given cluster and assign to type based on
///        variant used
struct EUCLIDEAN_CLUSTER_PUBLIC ComputeEigenBoxesWithZ
{
  explicit ComputeEigenBoxesWithZ(const autoware_auto_msgs::msg::PointClusters & clusters);
  void operator()(autoware_auto_msgs::msg::DetectedObjects & objects_msg);
  void operator()(autoware_auto_msgs::msg::BoundingBoxArray & boxes);

  const autoware_auto_msgs::msg::PointClusters & m_clusters;
};


}  // namespace euclidean_cluster
}  // namespace segmentation
}  // namespace perception
}  // namespace autoware

#endif   // EUCLIDEAN_CLUSTER__SHAPE_ESTIMATION_HPP_
