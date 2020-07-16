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

#ifndef TEST_RELATIVE_LOCALIZER_HPP_
#define TEST_RELATIVE_LOCALIZER_HPP_

#include <gtest/gtest.h>
#include <localization_common/localizer_base.hpp>

#include <string>

using autoware::localization::localization_common::LocalizerBase;

std::string merge_ids(int pose, int init, int map)
{
  return std::to_string(pose) + "_" + std::to_string(init) + std::to_string(map);
}

template<typename MsgT>
std::string get_id(const MsgT & msg)
{
  return msg.header.frame_id;
}
template<typename MsgT>
void set_id(MsgT & msg, const std::string & id)
{
  msg.header.frame_id = id;
}

class MockSummary {};

class TestLocalizer : public LocalizerBase<TestLocalizer>
{
public:
  geometry_msgs::msg::PoseWithCovarianceStamped register_measurement(
    const int & msg,
    const int & map,
    const geometry_msgs::msg::TransformStamped & transform_initial,
    MockSummary *)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_out{};
    set_id(pose_out, merge_ids(msg, std::stoi(get_id(transform_initial)), map));
    return pose_out;
  }
};

#endif  // TEST_RELATIVE_LOCALIZER_HPP_
