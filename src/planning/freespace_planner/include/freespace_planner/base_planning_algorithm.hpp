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

#ifndef FREESPACE_PLANNER__BASE_PLANNING_ALGORITHM_HPP_
#define FREESPACE_PLANNER__BASE_PLANNING_ALGORITHM_HPP_

#include <freespace_planner/visibility_control.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>

#include <cmath>
#include <functional>
#include <queue>
#include <string>
#include <tuple>
#include <vector>

#include "helper_functions/angle_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace autoware
{
namespace planning
{
namespace freespace_planner
{
// Wrap angle to the [min_rad, min_rad + 2PI] range.
double normalizeRadian(const double rad, const double min_rad = -M_PI);

int discretizeAngle(const double theta, const size_t theta_size);

struct IndexXYT
{
  int x;
  int y;
  int theta;
};

struct IndexXY
{
  int x;
  int y;
};

/// \brief Transform pose using given transform
geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform);


/// \brief Transform pose floating-point values to discrete costmap coordinates
IndexXYT pose2index(
  const geometry_msgs::msg::Pose & pose_local,
  const float & costmap_resolution,
  const size_t theta_size);

/// \brief Transform discrete costmap coordinates to floating-point pose values
geometry_msgs::msg::Pose index2pose(
  const IndexXYT & index, const float & costmap_resolution, const size_t theta_size);

/// \brief Transform pose from its frame to costmaps local frame
geometry_msgs::msg::Pose global2local(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_global);

/// \brief Transform pose from costmaps to its original frame
geometry_msgs::msg::Pose local2global(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_local);

/// \brief Definition of essential vehicle dimensions
struct FREESPACE_PLANNER_PUBLIC VehicleShape
{
  double length;   ///< Robot's length (bound with X axis direction) [m]
  double width;    ///< Robot's length (bound with Y axis direction)  [m]
  double cg2back;  ///< Robot's distance from center of gravity to back [m]
};

/// \brief Parameters defining common algorithm configuration
struct FREESPACE_PLANNER_PUBLIC PlannerCommonParam
{
  // base configs
  /// Planning time limit [msec]
  double time_limit;

  // robot configs
  /// Definition of robot shape
  VehicleShape vehicle_shape;
  /// Minimum possible turning radius to plan trajectory [m]
  double minimum_turning_radius;
  /// Maximum possible turning radius to plan trajectory [m]
  double maximum_turning_radius;
  /// Number of levels of discretization between minimum and maximum turning radius [-]
  size_t turning_radius_size;

  // search configs
  /// Number of possible headings, discretized between <0, 2pi> [-]
  size_t theta_size;
  /// Cost of changing moving direction [-]
  double reverse_weight;
  /// Lateral tolerance of goal pose [m]
  double goal_lateral_tolerance;
  /// Longitudinal tolerance of goal pose [m]
  double goal_longitudinal_tolerance;
  /// Angular tolerance of goal pose [rad]
  double goal_angular_tolerance;

  // costmap configs
  /// Threshold value of costmap cell to be regarded as an obstacle [-]
  int64_t obstacle_threshold;
};

struct PlannerWaypoint
{
  geometry_msgs::msg::PoseStamped pose;
  bool is_back = false;
};

/// \brief Trajectory points representation as an algorithms output
struct FREESPACE_PLANNER_PUBLIC PlannerWaypoints
{
  std_msgs::msg::Header header;            ///< Mostly timestamp and frame information
  std::vector<PlannerWaypoint> waypoints;  ///< Vector of trajectory waypoints
};

/// \brief Possible planning results
enum class SearchStatus
{
  SUCCESS,                     ///< Planning successful
  FAILURE_COLLISION_AT_START,  ///< Collision at start position detected
  FAILURE_COLLISION_AT_GOAL,   ///< Collision at goal position detected
  FAILURE_TIMEOUT_EXCEEDED,    ///< Planning timeout exceeded
  FAILURE_NO_PATH_FOUND        ///< Planner didn't manage to find path
};

/// \brief Determines if passed status is a success status
inline bool FREESPACE_PLANNER_PUBLIC isSuccess(const SearchStatus & status)
{
  return status == SearchStatus::SUCCESS;
}

/// \class BasePlanningAlgorithm
/// \brief Planning algorithm base class
class FREESPACE_PLANNER_PUBLIC BasePlanningAlgorithm
{
public:
  /// \brief Class constructor
  /// \param[in] planner_common_param PlannerCommonParam object
  explicit BasePlanningAlgorithm(const PlannerCommonParam & planner_common_param)
  : planner_common_param_(planner_common_param)
  {
  }

  /// \brief Robot dimensions setter
  /// \param[in] vehicle_shape VehicleShape object
  virtual void setVehicleShape(const VehicleShape & vehicle_shape)
  {
    planner_common_param_.vehicle_shape = vehicle_shape;
  }

  /// \brief Set occupancy grid for planning
  /// \param[in] costmap nav_msgs::msg::OccupancyGrid type object
  virtual void setOccupancyGrid(const nav_msgs::msg::OccupancyGrid & costmap);

  /// \brief Create trajectory plan
  /// \param[in] start_pose Start position
  /// \param[in] goal_pose Goal position
  /// \return SearchStatus flag showing if planning succeeded or not
  virtual SearchStatus makePlan(
    const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose) = 0;

  /// \brief Check if there will be collision on generated trajectory
  /// \param[in] trajectory Generated trajectory
  /// \return True if detected collision
  bool hasObstacleOnTrajectory(const geometry_msgs::msg::PoseArray & trajectory) const;

  /// \brief Fetch algorithm solution
  /// \return PlannerWaypoints created trajectory
  const PlannerWaypoints & getWaypoints() const {return waypoints_;}

  virtual ~BasePlanningAlgorithm() {}

protected:
  void computeCollisionIndexes(int theta_index, std::vector<IndexXY> & indexes);
  bool detectCollision(const IndexXYT & base_index) const;
  inline bool isOutOfRange(const IndexXYT & index) const
  {
    if (index.x < 0 || static_cast<int>(costmap_.info.width) <= index.x) {
      return true;
    }
    if (index.y < 0 || static_cast<int>(costmap_.info.height) <= index.y) {
      return true;
    }
    return false;
  }

  inline bool isObs(const IndexXYT & index) const
  {
    // NOTE: Accessing by .at() instead makes 1.2 times slower here.
    // Also, boundary check is already done in isOutOfRange before calling this function.
    return is_obstacle_table_[static_cast<size_t>(index.y)][static_cast<size_t>(index.x)];
  }

  PlannerCommonParam planner_common_param_;
  nav_msgs::msg::OccupancyGrid costmap_;
  std::vector<std::vector<IndexXY>> coll_indexes_table_;
  std::vector<std::vector<bool>> is_obstacle_table_;

  // pose in costmap frame
  geometry_msgs::msg::Pose start_pose_;
  geometry_msgs::msg::Pose goal_pose_;

  // result path
  PlannerWaypoints waypoints_;
};

}  // namespace freespace_planner
}  // namespace planning
}  // namespace autoware

#endif  // FREESPACE_PLANNER__BASE_PLANNING_ALGORITHM_HPP_
