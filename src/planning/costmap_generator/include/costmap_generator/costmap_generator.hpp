// Copyright 2020 Tier IV, Inc.
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

/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************/

#ifndef COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
#define COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "lanelet2_core/primitives/Lanelet.h"

#include "std_msgs/msg/bool.hpp"
#include "autoware_auto_msgs/srv/had_map_service.hpp"
#include "autoware_auto_msgs/action/planner_costmap.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


namespace autoware
{
namespace planning
{
namespace costmap_generator
{

enum class CostmapGeneratorState
{
  IDLE,
  GENERATING
};

class CostmapGenerator : public rclcpp::Node
{
public:
  explicit CostmapGenerator(const rclcpp::NodeOptions & node_options);

private:
  bool use_wayarea_;

  std::string costmap_frame_;
  std::string vehicle_frame_;
  std::string map_frame_;

  double grid_min_value_;
  double grid_max_value_;
  double grid_resolution_;
  double grid_length_x_;
  double grid_length_y_;
  double grid_position_x_;
  double grid_position_y_;

  double route_box_padding_;

  grid_map::GridMap costmap_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_occupancy_grid_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_local_had_map_publisher_;

  rclcpp_action::Server<autoware_auto_msgs::action::PlannerCostmap>::SharedPtr costmap_action_server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<autoware_auto_msgs::action::PlannerCostmap>> goal_handle_{nullptr};

  rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedPtr map_client_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<std::vector<geometry_msgs::msg::Point>> area_points_;

  struct LayerName
  {
    static constexpr const char * WAYAREA = "wayarea";
    static constexpr const char * COMBINED = "combined";
  };

  // action
  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const autoware_auto_msgs::action::PlannerCostmap::Goal>);
  rclcpp_action::CancelResponse handleCancel(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<autoware_auto_msgs::action::PlannerCostmap>>);
  void handleAccepted(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<autoware_auto_msgs::action::PlannerCostmap>>);

  void setCostmapPositionAtVehiclePosition();

  std::tuple<grid_map::Position, grid_map::Position> calculateAreaPointsBoundingBox() const;

  grid_map::GridMap boundCostmap(const grid_map::Position &, const grid_map::Position &) const;

  /// \brief create map request for driveable area with bounds that are based on route
  autoware_auto_msgs::srv::HADMapService::Request
  createMapRequest(const autoware_auto_msgs::msg::HADMapRoute &) const;

  /// \brief callback for HAD map that composes costmap and successes action
  void mapResponse(rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedFuture future);

  /// \brief initialize gridmap parameters based on rosparam
  void initGridmap();

  /// \brief create occupancy grid based on input grid map and parameters
  /// \param [in] input grid map
  /// \param [out] converted occupancy grid
  nav_msgs::msg::OccupancyGrid createOccupancyGrid(const grid_map::GridMap &) const;

  /// \brief set area_points_ from lanelet polygons
  /// \param [in] input lanelet_map
  void loadRoadAreasFromLaneletMap(lanelet::LaneletMapPtr);

  /// \brief set area_points_ from parking areas
  /// \param [in] input lanelet_map
  void loadParkingAreasFromLaneletMap(lanelet::LaneletMapPtr);

  /// \brief calculate cost from lanelet2 map
  grid_map::Matrix generateWayAreaCostmap() const;

  /// \brief calculate cost for final output
  grid_map::Matrix generateCombinedCostmap() const;

  /// \brief generate all layers of costmap
  void generateCostmapLayers();

  /// \brief publish visualization of received part of the HAD map
  void publishLaneletVisualization(std::shared_ptr<lanelet::LaneletMap> & map);

  CostmapGeneratorState state_;
  bool isIdle() const { return state_ == CostmapGeneratorState::IDLE; }
  bool isGenerating() const { return state_ == CostmapGeneratorState::GENERATING; }
  void setIdleState() { state_ = CostmapGeneratorState::IDLE; }
  void setGeneratingState() { state_ = CostmapGeneratorState::GENERATING; }
};

}  // namespace autoware
}  // namespace planning
}  // namespace costmap_generator

#endif  // COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
