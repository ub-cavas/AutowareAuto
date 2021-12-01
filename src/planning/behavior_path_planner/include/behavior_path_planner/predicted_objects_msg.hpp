// Copyright 2021 Tier IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER_PREDICTED_OBJECTS_MSG_HPP_
#define BEHAVIOR_PATH_PLANNER_PREDICTED_OBJECTS_MSG_HPP_

#include "autoware_auto_perception_msgs/msg/object_classification.hpp"
#include "autoware_auto_perception_msgs/msg/shape.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/accel_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// This header contains a customized copy of predicted_objects.idl structure to
// ease the usage of the message in the ported package.
namespace behavior_path_planner
{

struct PredictedPath{
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> path;
  float confidence;
};

struct PredictedObjectKinematics{
  geometry_msgs::msg::PoseWithCovariance initial_pose;
  geometry_msgs::msg::TwistWithCovariance initial_twist;
  geometry_msgs::msg::AccelWithCovariance initial_acceleration;
  std::vector<PredictedPath> predicted_paths;
};

struct PredictedObject{
  uint64_t object_id;
  float existence_probability;
  std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classification;
  PredictedObjectKinematics kinematics;
  std::vector<autoware_auto_perception_msgs::msg::Shape> shape;
};

struct PredictedObjects{
  std_msgs::msg::Header header;
  std::vector<PredictedObject> objects;
};

} //namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER_PREDICTED_OBJECTS_MSG_HPP_
