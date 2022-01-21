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

#ifndef TEST_CONVEX_HULL_COMPUTATION_HPP_
#define TEST_CONVEX_HULL_COMPUTATION_HPP_

#include <euclidean_cluster/euclidean_cluster.hpp>

#include <vector>

#include "gtest/gtest.h"

using Clusters = autoware_auto_perception_msgs::msg::PointClusters;
using DetectedObjects = autoware_auto_perception_msgs::msg::DetectedObjects;
using DetectedObject = autoware_auto_perception_msgs::msg::DetectedObject;
using Pt = autoware_auto_perception_msgs::msg::PointXYZIF;

using autoware::perception::segmentation::euclidean_cluster::details::convert_to_polygon_prisms;

class ConvexHullComputationTest : public ::testing::Test
{
protected:
  Pt make_pt(const float x, const float y, const float z = 0.0F)
  {
    Pt ret;
    ret.x = x;
    ret.y = y;
    ret.z = z;
    return ret;
  }

  Clusters make_clusters(std::vector<std::vector<Pt>> points_list)
  {
    Clusters ret;
    size_t boundary_idx = 0U;
    for (const auto & list : points_list) {
      boundary_idx += list.size();
      for (const auto & pt : list) {
        ret.points.push_back(pt);
      }
      ret.cluster_boundary.push_back(boundary_idx);
    }
    return ret;
  }

  void test_convex_hull_object(
    const DetectedObject & object, const std::vector<Pt> & expect,
    const float TOL = 1.0E-6F)
  {
    ASSERT_EQ(object.shape.polygon.points.size(), expect.size());

    for (uint32_t idx = 0U; idx < expect.size(); ++idx) {
      auto & actual = object.shape.polygon.points[idx];
      auto x_pc_frame = actual.x + static_cast<float32_t>(
        object.kinematics.pose_with_covariance.pose.position.x);
      auto y_pc_frame = actual.y + static_cast<float32_t>(
        object.kinematics.pose_with_covariance.pose.position.y);
      auto z_pc_frame = actual.z + static_cast<float32_t>(
        object.kinematics.pose_with_covariance.pose.position.z);

      bool found = false;
      for (auto & p : expect) {
        if (fabsf(p.x - x_pc_frame) < TOL && fabsf(p.y - y_pc_frame) < TOL &&
          fabsf(p.z - z_pc_frame) < TOL)
        {
          found = true;
          break;
        }
      }
      ASSERT_TRUE(found) << idx << ": " << actual.x << ", " << actual.y;
    }

    EXPECT_EQ(object.existence_probability, 1.0F);
    ASSERT_EQ(object.classification.size(), 1U);
    EXPECT_EQ(
      object.classification[0U].classification,
      autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN);
    EXPECT_EQ(object.classification[0U].probability, 1.0F);
  }


  //  The point cloud is visualized in the link.
  //  https://chart-studio.plotly.com/~kaancolak/3
  std::vector<Pt> pt_vector{make_pt(-0.0F, 0.0F), make_pt(0.0F, 0.0F), make_pt(0.0F, 2.0F),
    make_pt(0.0F, -2.0F), make_pt(1.0F, 2.0F), make_pt(0.0F, -3.0F), make_pt(1.0F, -1.0F),
    make_pt(1.5F, 2.0F), make_pt(0.F, 5.0F), make_pt(-2.0F, 2.0F), make_pt(-2.0F, -1.0F),
    make_pt(0.F, -3.0F), make_pt(0.F, -3.0F), make_pt(-1.0F, 0.0F)};

  std::vector<Pt> convex_hull_expected_corners_pc_frame{
    make_pt(0.0F, -3.0F), make_pt(1.0F, -1.0F), make_pt(1.5F, 2.0F),
    make_pt(0.F, 5.0F), make_pt(-2.0F, 2.0F), make_pt(-2.0F, -1.0F)};
};


TEST_F(ConvexHullComputationTest, TestConvexHull2D)
{
  auto clusters = make_clusters(
    {pt_vector, pt_vector});

  DetectedObjects boxes_msg = convert_to_polygon_prisms(clusters);
  ASSERT_EQ(boxes_msg.objects.size(), 2U);
  for (const auto & box : boxes_msg.objects) {
    test_convex_hull_object(box, convex_hull_expected_corners_pc_frame, 1E-3F);
  }
}


#endif   // TEST_CONVEX_HULL_COMPUTATION_HPP_
