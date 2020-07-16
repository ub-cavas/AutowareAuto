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

#include <gtest/gtest.h>
#include <localization_common/localizer_base.hpp>
#include "test_relative_localizer.hpp"

using Transform = geometry_msgs::msg::TransformStamped;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

class TestRelativeLocalizerBase : public ::testing::Test
{
protected:
  void SetUp()
  {
    set_id(m_init, std::to_string(m_init_id));
  }

  Transform m_init;
  const int m_pose_id{3};
  const int m_init_id{5};
  const int m_map_id{7};
};

TEST_F(TestRelativeLocalizerBase, basic_io) {
  TestLocalizer localizer;
  PoseWithCovarianceStamped dummy_pose;

  PoseWithCovarianceStamped pose_out{};
  EXPECT_NO_THROW(pose_out = localizer.register_measurement(m_pose_id, m_map_id, m_init, nullptr));

  EXPECT_EQ(get_id(pose_out), merge_ids(m_pose_id, m_init_id, m_map_id));
}
