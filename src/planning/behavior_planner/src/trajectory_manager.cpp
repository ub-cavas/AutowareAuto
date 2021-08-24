// Copyright 2020 The Autoware Foundation
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

#include "behavior_planner/trajectory_manager.hpp"
#include <geometry/common_2d.hpp>
#include <motion_common/motion_common.hpp>

#include <algorithm>
#include <limits>
#include <vector>

namespace autoware
{
namespace behavior_planner
{

using autoware::common::geometry::distance_2d;
using autoware::common::geometry::minus_2d;
using autoware::common::geometry::norm_2d;
using motion::motion_common::to_angle;

TrajectoryManager::TrajectoryManager(const PlannerConfig & config)
: m_config(config)
{
}

void TrajectoryManager::clear_trajectory()
{
  m_trajectory.points.clear();
  m_sub_trajectories.clear();
  m_selected_trajectory = 0;
}

void TrajectoryManager::set_trajectory(const Trajectory & trajectory)
{
  clear_trajectory();
  m_trajectory = trajectory;
  set_sub_trajectories();
}

void TrajectoryManager::set_sub_trajectories()
{
  // return sign with hysterysis buffer
  const auto is_positive = [](const TrajectoryPoint & pt) {
      // using epsilon instead to ensure change in sign.
      static bool8_t is_prev_positive = true;
      bool8_t is_positive;
      if (is_prev_positive) {
        is_positive = pt.longitudinal_velocity_mps > -std::numeric_limits<float32_t>::epsilon();
      } else {
        is_positive = pt.longitudinal_velocity_mps > std::numeric_limits<float32_t>::epsilon();
      }
      is_prev_positive = is_positive;
      return is_positive;
    };

  if (m_trajectory.points.empty()) {
    m_sub_trajectories.push_back(m_trajectory);
    return;
  }

  Trajectory sub_trajectory;
  sub_trajectory.header = m_trajectory.header;

  auto prev_sign = is_positive(m_trajectory.points.front());
  for (auto & pt : m_trajectory.points) {
    const auto sign = is_positive(pt);
    if (prev_sign != sign && !sub_trajectory.points.empty()) {
      m_sub_trajectories.push_back(sub_trajectory);
      sub_trajectory.points.clear();
    }
    sub_trajectory.points.push_back(pt);
    prev_sign = sign;
  }
  if (!sub_trajectory.points.empty()) {
    m_sub_trajectories.push_back(sub_trajectory);
  }
}

bool8_t TrajectoryManager::is_trajectory_ready()
{
  return !m_sub_trajectories.empty();
}

std::size_t TrajectoryManager::get_closest_state(
  const State & current_state,
  const Trajectory & trajectory)
{
  const auto distance_from_current_state =
    [this, &current_state](const TrajectoryPoint & other_state) {
      const auto s1 = current_state.state, s2 = other_state;
      const auto distance = norm_2d(minus_2d(s1, s2));
      const auto angle_diff = std::abs(to_angle(s1.heading - s2.heading));
      return distance + m_config.heading_weight * angle_diff;
    };

  const auto comparison_function =
    [&distance_from_current_state](const TrajectoryPoint & one, const TrajectoryPoint & two)
    {return distance_from_current_state(one) < distance_from_current_state(two);};

  const auto minimum_index_iterator =
    std::min_element(
    std::begin(trajectory.points), std::end(trajectory.points),
    comparison_function);
  auto minimum_idx = std::distance(std::begin(trajectory.points), minimum_index_iterator);

  return static_cast<std::size_t>(minimum_idx);
}

size_t TrajectoryManager::get_remaining_length(const State & state)
{
  // remaining length of current selected sub trajectory
  const auto & current_trajectory = m_sub_trajectories.at(m_selected_trajectory);
  const size_t closest_index = get_closest_state(state, current_trajectory);
  size_t remaining_length = current_trajectory.points.size() - closest_index;

  // remaining length including rest of sub trajectories
  for (size_t i = m_selected_trajectory + 1; i < m_sub_trajectories.size(); i++) {
    remaining_length += m_sub_trajectories.at(i).points.size();
  }

  return remaining_length;
}

void TrajectoryManager::set_time_from_start(Trajectory * trajectory, const size_t start_index)
{
  if (trajectory->points.empty()) {
    return;
  }

  float32_t t = 0.0;

  // special operation for first point
  auto & first_point = trajectory->points.at(start_index);
  first_point.time_from_start.sec = 0;
  first_point.time_from_start.nanosec = 0;

  for (std::size_t i = start_index + 1; i < trajectory->points.size(); ++i) {
    auto & p0 = trajectory->points[i - 1];
    auto & p1 = trajectory->points[i];
    auto v = 0.5f * (p0.longitudinal_velocity_mps + p1.longitudinal_velocity_mps);
    t += norm_2d(minus_2d(p0, p1)) / std::max(std::fabs(v), 0.5f);
    float32_t t_s = 0;
    float32_t t_ns = std::modf(t, &t_s) * 1.0e9f;
    trajectory->points[i].time_from_start.sec = static_cast<int32_t>(t_s);
    trajectory->points[i].time_from_start.nanosec = static_cast<uint32_t>(t_ns);
  }
}

float32_t TrajectoryManager::generate_previous_points(
  const size_t from_index,
  std::vector<TrajectoryPoint> & previous_points)
{
  previous_points.clear();
  TrajectoryPoint prev_point =
    m_sub_trajectories[m_selected_trajectory].points[from_index];
  bool8_t end_reached = false;
  float32_t dist = 0.0;
  size_t current_traj_idx = m_selected_trajectory;
  size_t current_point_idx = from_index;
  // start from the point before the given index
  while (current_point_idx == 0) {
    --current_traj_idx;
    current_point_idx = m_sub_trajectories[current_traj_idx].points.size();
  }
  --current_point_idx;

  while (!end_reached && dist < m_config.prepend_distance) {
    const auto & curr_point = m_sub_trajectories[current_traj_idx].points[current_point_idx];
    dist += distance_2d<float32_t>(prev_point, curr_point);
    previous_points.push_back(curr_point);
    prev_point = curr_point;

    // Reached the first point of the sub trajectory
    while (current_point_idx == 0) {
      // Reached the last sub trajectory: no more previous points
      if (current_traj_idx == 0) {
        end_reached = true;
        break;
      }
      --current_traj_idx;
      current_point_idx = m_sub_trajectories[current_traj_idx].points.size();
    }

    --current_point_idx;
  }
  return dist;
}

bool TrajectoryManager::extrapolate(
  const size_t first_index,
  const std::vector<TrajectoryPoint> & previous_points,
  const float32_t extra_dist, TrajectoryPoint & extra_point)
{
  TrajectoryPoint first;
  TrajectoryPoint second;
  // First point: first previous point or point from first_index
  if (previous_points.size() > 0) {
    first = previous_points[0];
  } else {
    first = m_sub_trajectories[m_selected_trajectory].points[first_index];
  }
  // Second point: second previous point or point after first_index
  if (previous_points.size() > 1) {
    second = previous_points[1];
  } else {
    if (m_sub_trajectories[m_selected_trajectory].points.size() > first_index + 1) {
      second = m_sub_trajectories[m_selected_trajectory].points[first_index + 1];
    } else if (m_sub_trajectories.size() > m_selected_trajectory + 1) {
      second = m_sub_trajectories[m_selected_trajectory + 1].points[0];
    } else {
      return false;
    }
  }
  const auto dist = norm_2d(minus_2d(first, second));
  extra_point = first;
  extra_point.x = first.x - (extra_dist / dist) * (second.x - first.x);
  extra_point.y = first.y - (extra_dist / dist) * (second.y - first.y);
  return true;
}

Trajectory TrajectoryManager::get_trajectory(const State & state)
{
  const size_t capacity = static_cast<size_t>(Trajectory::CAPACITY);
  // select new sub_trajectory when vehicle is at stop
  if (std::abs(state.state.longitudinal_velocity_mps) < m_config.stop_velocity_thresh) {
    const auto & last_point = m_sub_trajectories.at(m_selected_trajectory).points.back();
    const auto distance = norm_2d(minus_2d(last_point, state.state));

    // increment index to select new sub_trajectory if vehicle has arrived the end of sub_trajectory
    if (distance < m_config.goal_distance_thresh) {
      m_selected_trajectory++;
      m_selected_trajectory = std::min(m_selected_trajectory, m_sub_trajectories.size() - 1);
    }
  }

  // TODO(mitsudome-r) implement trajectory refine functions if needed to integrate with controller
  const Trajectory & sub_trajectory = m_sub_trajectories[m_selected_trajectory];
  Trajectory output_trajectory;
  output_trajectory.header = sub_trajectory.header;
  const size_t closest_state_index = get_closest_state(state, sub_trajectory);

  if (m_config.prepend_distance > 0) {
    std::vector<TrajectoryPoint> previous_points;
    float32_t remaining_prep_dist = m_config.prepend_distance;
    if (closest_state_index > 0 || m_selected_trajectory > 0) {
      remaining_prep_dist -= generate_previous_points(closest_state_index, previous_points);
    }
    // if there are not enough points to reach the desired distance, extrapolate
    if (remaining_prep_dist > 0) {
      TrajectoryPoint extra_point;
      if (extrapolate(closest_state_index, previous_points, remaining_prep_dist, extra_point)) {
        previous_points.push_back(extra_point);
      }
    }
    previous_points.resize(std::min(previous_points.size(), capacity));
    output_trajectory.points.insert(
      output_trajectory.points.begin(),
      previous_points.rbegin(), previous_points.rend());
  }

  if (m_config.include_current_state && output_trajectory.points.size() < capacity) {
    output_trajectory.points.push_back(state.state);
    output_trajectory.points.back().longitudinal_velocity_mps =
      sub_trajectory.points.at(closest_state_index).longitudinal_velocity_mps;
  }
  const size_t start_index = output_trajectory.points.size();
  // add trajectory points after the current state
  for (size_t i = closest_state_index + 1;
    i < sub_trajectory.points.size() && output_trajectory.points.size() < capacity; i++)
  {
    output_trajectory.points.push_back(sub_trajectory.points[i]);
  }

  set_time_from_start(
    &output_trajectory,
    std::min(start_index, output_trajectory.points.size() - 1));
  return output_trajectory;
}

}  // namespace behavior_planner
}  // namespace autoware
