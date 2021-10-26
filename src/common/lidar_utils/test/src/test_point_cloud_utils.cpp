// Copyright 2017-2021 the Autoware Foundation, Arm Limited
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

#include <common/types.hpp>
#include <lidar_utils/lidar_utils.hpp>
#include <lidar_utils/point_cloud_utils.hpp>

#include <gtest/gtest.h>

#include <string>
#include <vector>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

TEST(TestStaticTransformer, TransformPoint)
{
  Eigen::Quaternionf rotation;
  rotation = Eigen::AngleAxisf(M_PI_2f32, Eigen::Vector3f::UnitZ());

  geometry_msgs::msg::Transform tf;
  tf.translation.x = 1.0;
  tf.translation.y = 2.0;
  tf.translation.z = 3.0;
  // Rotation around z axis by 90 degrees.
  tf.rotation.x = static_cast<float64_t>(rotation.x());
  tf.rotation.y = static_cast<float64_t>(rotation.y());
  tf.rotation.z = static_cast<float64_t>(rotation.z());
  tf.rotation.w = static_cast<float64_t>(rotation.w());
  autoware::common::types::PointXYZF point{5.0F, 5.0F, 5.0F};
  autoware::common::lidar_utils::StaticTransformer transformer{tf};
  autoware::common::types::PointXYZF result_point{};
  transformer.transform(point, result_point);
  EXPECT_FLOAT_EQ(-4.0F, result_point.x);
  EXPECT_FLOAT_EQ(7.0F, result_point.y);
  EXPECT_FLOAT_EQ(8.0F, result_point.z);
}
