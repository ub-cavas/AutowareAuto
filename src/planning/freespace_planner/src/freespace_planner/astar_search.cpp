// Copyright 2021 The Autoware Foundation
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
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.


#include "freespace_planner/astar_search.hpp"

#include <vector>

#include "tf2/utils.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wuseless-cast"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#pragma GCC diagnostic pop


namespace autoware
{
namespace planning
{
namespace freespace_planner
{
constexpr double deg2rad(const double deg)
{
  return deg * M_PI / 180.0;
}

double calcReedsSheppDistance(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2, double radius)
{
  auto rs_space = ReedsShepp(radius);
  StateXYT pose0{p1.position.x, p1.position.y, tf2::getYaw(p1.orientation)};
  StateXYT pose1{p2.position.x, p2.position.y, tf2::getYaw(p2.orientation)};
  return rs_space.distance(pose0, pose1);
}

double calcDistance2d(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

double calcDistance2d(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2)
{
  return calcDistance2d(p1.position, p2.position);
}

geometry_msgs::msg::Quaternion makeQuaternionWithYaw(const double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quat);
}

geometry_msgs::msg::Pose calcRelativePose(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Pose & pose)
{
  tf2::Transform tf_transform;
  tf2::convert(base_pose, tf_transform);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_transform.inverse());

  geometry_msgs::msg::PoseStamped transformed;
  geometry_msgs::msg::PoseStamped pose_orig;
  pose_orig.pose = pose;
  tf2::doTransform(pose_orig, transformed, transform);

  return transformed.pose;
}

geometry_msgs::msg::Pose node2pose(const AstarNode & node)
{
  geometry_msgs::msg::Pose pose_local;

  pose_local.position.x = node.x;
  pose_local.position.y = node.y;
  pose_local.position.z = 0;

  tf2::Quaternion quat;
  quat.setRPY(0, 0, node.theta);
  tf2::convert(quat, pose_local.orientation);

  return pose_local;
}

AstarSearch::TransitionTable createTransitionTable(
  const double minimum_turning_radius,
  const double maximum_turning_radius,
  const size_t turning_radius_size,
  const size_t theta_size,
  const bool use_back)
{
  // Vehicle moving for each angle
  AstarSearch::TransitionTable transition_table;
  transition_table.resize(theta_size);

  const double dtheta = 2.0 * M_PI / static_cast<double>(theta_size);

  // Minimum moving distance with one state update
  // arc  = r * theta
  const auto & R_min = minimum_turning_radius;
  const auto & R_max = maximum_turning_radius;
  const double step_min = R_min * dtheta;
  const double dR = (R_max - R_min) / static_cast<double>(turning_radius_size);

  // NodeUpdate actions
  std::vector<NodeUpdate> forward_node_candidates;
  const NodeUpdate forward_straight{step_min, 0.0, 0.0, step_min, false};
  forward_node_candidates.push_back(forward_straight);
  for (int i = 0; i < static_cast<int>(turning_radius_size + 1); ++i) {
    double R = R_min + i * dR;
    double step = R * dtheta;
    NodeUpdate forward_left{R * sin(dtheta), R * (1 - cos(dtheta)), dtheta, step, false};
    NodeUpdate forward_right = forward_left.flipped();
    forward_node_candidates.push_back(forward_left);
    forward_node_candidates.push_back(forward_right);
  }

  for (size_t i = 0; i < theta_size; i++) {
    const double theta = dtheta * static_cast<double>(i);

    for (const auto & nu : forward_node_candidates) {
      transition_table[i].push_back(nu.rotated(theta));
    }

    if (use_back) {
      for (const auto & nu : forward_node_candidates) {
        transition_table[i].push_back(nu.reversed().rotated(theta));
      }
    }
  }

  return transition_table;
}

AstarSearch::AstarSearch(
  const PlannerCommonParam & planner_common_param, const AstarParam & astar_param)
: BasePlanningAlgorithm(planner_common_param), astar_param_(astar_param)
{
  transition_table_ = createTransitionTable(
    planner_common_param_.minimum_turning_radius,
    planner_common_param_.maximum_turning_radius,
    planner_common_param_.turning_radius_size,
    planner_common_param_.theta_size,
    astar_param_.use_back);
}

SearchStatus AstarSearch::makePlan(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose)
{
  start_pose_ = global2local(costmap_, start_pose);
  goal_pose_ = global2local(costmap_, goal_pose);

  if (!setStartNode()) {
    return SearchStatus::FAILURE_COLLISION_AT_START;
  }

  if (!setGoalNode()) {
    return SearchStatus::FAILURE_COLLISION_AT_GOAL;
  }

  return search();
}

bool AstarSearch::setStartNode()
{
  const auto index = pose2index(
    start_pose_, costmap_.info.resolution, planner_common_param_.theta_size);

  if (detectCollision(index)) {
    return false;
  }

  // Set start node
  AstarNode * start_node = getNodeRef(index);
  start_node->x = start_pose_.position.x;
  start_node->y = start_pose_.position.y;
  start_node->theta = 2.0 * M_PI / static_cast<double>(planner_common_param_.theta_size) *
    static_cast<double>(index.theta);
  start_node->gc = 0;
  start_node->hc = estimateCost(start_pose_);
  start_node->is_back = false;
  start_node->status = NodeStatus::Open;
  start_node->parent = nullptr;

  // Push start node to openlist
  openlist_.push(start_node);

  return true;
}

bool AstarSearch::setGoalNode() const
{
  const auto index = pose2index(
    goal_pose_, costmap_.info.resolution, planner_common_param_.theta_size);

  if (detectCollision(index)) {
    return false;
  }

  return true;
}

double AstarSearch::estimateCost(const geometry_msgs::msg::Pose & pose) const
{
  double total_cost = 0.0;
  // Temporarily, until reeds_shepp gets stable.
  if (astar_param_.use_reeds_shepp) {
    double radius = (planner_common_param_.minimum_turning_radius +
      planner_common_param_.maximum_turning_radius) *
      0.5;
    total_cost +=
      calcReedsSheppDistance(pose, goal_pose_, radius) * astar_param_.distance_heuristic_weight;
  } else {
    total_cost += calcDistance2d(pose, goal_pose_) * astar_param_.distance_heuristic_weight;
  }
  return total_cost;
}

void AstarSearch::setOccupancyGrid(const nav_msgs::msg::OccupancyGrid & costmap)
{
  BasePlanningAlgorithm::setOccupancyGrid(costmap);
  const auto height = costmap_.info.height;
  const auto width = costmap_.info.width;

  // Initialize nodes
  nodes_.clear();
  nodes_.resize(height);
  for (size_t i = 0; i < height; i++) {
    nodes_[i].resize(width);
    for (size_t j = 0; j < width; j++) {
      nodes_[i][j].resize(planner_common_param_.theta_size);
    }
  }
}

SearchStatus AstarSearch::search()
{
  const rclcpp::Time begin = rclcpp::Clock(RCL_ROS_TIME).now();

  // Start A* search
  while (!openlist_.empty()) {
    // Check time and terminate if the search reaches the time limit
    const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
    const double msec = (now - begin).seconds() * 1000.0;
    if (msec > planner_common_param_.time_limit) {
      return SearchStatus::FAILURE_TIMEOUT_EXCEEDED;
    }

    // Expand minimum cost node
    AstarNode * current_node = openlist_.top();
    openlist_.pop();
    current_node->status = NodeStatus::Closed;

    if (isGoal(*current_node)) {
      setPath(*current_node);
      return SearchStatus::SUCCESS;
    }

    // Transit
    const auto index_theta = discretizeAngle(current_node->theta, planner_common_param_.theta_size);
    for (const auto & transition : transition_table_[static_cast<size_t>(index_theta)]) {
      const bool is_turning_point = transition.is_back != current_node->is_back;

      // TODO(T.Horibe): change step to distance (just rename)
      const double move_cost =
        is_turning_point ? planner_common_param_.reverse_weight * transition.step : transition.step;

      // Calculate index of the next state
      geometry_msgs::msg::Pose next_pose;
      next_pose.position.x = current_node->x + transition.shift_x;
      next_pose.position.y = current_node->y + transition.shift_y;
      next_pose.orientation = makeQuaternionWithYaw(current_node->theta + transition.shift_theta);
      const auto next_index = pose2index(
        next_pose, costmap_.info.resolution, planner_common_param_.theta_size);

      if (detectCollision(next_index)) {
        continue;
      }

      // Compare cost
      AstarNode * next_node = getNodeRef(next_index);
      const double next_gc = current_node->gc + move_cost;
      if (next_node->status == NodeStatus::None || next_gc < next_node->gc) {
        next_node->status = NodeStatus::Open;
        next_node->x = next_pose.position.x;
        next_node->y = next_pose.position.y;
        next_node->theta = tf2::getYaw(next_pose.orientation);
        next_node->gc = next_gc;
        next_node->hc = estimateCost(next_pose);
        next_node->is_back = transition.is_back;
        next_node->parent = current_node;
        openlist_.push(next_node);
        continue;
      }
    }
  }

  // Failed to find path
  return SearchStatus::FAILURE_NO_PATH_FOUND;
}

void AstarSearch::setPath(const AstarNode & goal_node)
{
  std_msgs::msg::Header header;
  header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  header.frame_id = costmap_.header.frame_id;

  waypoints_.header = header;
  waypoints_.waypoints.clear();

  // From the goal node to the start node
  const AstarNode * node = &goal_node;

  while (node != nullptr) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose = local2global(costmap_, node2pose(*node));

    // PlannerWaypoints
    PlannerWaypoint pw;
    pw.pose = pose;
    pw.is_back = node->is_back;
    waypoints_.waypoints.push_back(pw);

    // To the next node
    node = node->parent;
  }

  // Reverse the vector to be start to goal order
  std::reverse(waypoints_.waypoints.begin(), waypoints_.waypoints.end());

  // Update first point direction
  if (waypoints_.waypoints.size() > 1) {
    waypoints_.waypoints.at(0).is_back = waypoints_.waypoints.at(1).is_back;
  }
}

bool AstarSearch::isGoal(const AstarNode & node) const
{
  const auto relative_pose = calcRelativePose(goal_pose_, node2pose(node));

  // Check conditions
  if (astar_param_.only_behind_solutions && relative_pose.position.x > 0) {
    return false;
  }

  if (
    std::fabs(relative_pose.position.x) > planner_common_param_.goal_longitudinal_tolerance ||
    std::fabs(relative_pose.position.y) > planner_common_param_.goal_lateral_tolerance)
  {
    return false;
  }

  const auto angle_diff = normalizeRadian(tf2::getYaw(relative_pose.orientation));
  if (std::abs(angle_diff) > planner_common_param_.goal_angular_tolerance) {
    return false;
  }

  return true;
}

AstarNode * AstarSearch::getNodeRef(const IndexXYT & index)
{
  return &nodes_[static_cast<size_t>(index.y)][static_cast<size_t>(index.x)]
         [static_cast<size_t>(index.theta)];
}

}  // namespace freespace_planner
}  // namespace planning
}  // namespace autoware
