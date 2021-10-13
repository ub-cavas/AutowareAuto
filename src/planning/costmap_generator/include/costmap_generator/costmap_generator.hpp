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

#include <costmap_generator/visibility_control.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <lanelet2_core/primitives/Lanelet.h>

#include <std_msgs/msg/bool.hpp>
#include <autoware_auto_msgs/srv/had_map_service.hpp>
#include <autoware_auto_msgs/action/planner_costmap.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


namespace autoware
{
namespace planning
{
namespace costmap_generator
{

/// \brief Enum used for setting CostmapGenerator class state
enum class CostmapGeneratorState
{
  IDLE,
  GENERATING
};

/// \class CostmapGenerator
/// \brief Creates costmap by combining lanelet2 OSM map and information about obstacles
class COSTMAP_GENERATOR_PUBLIC CostmapGenerator : public rclcpp::Node
{
public:
  /// \brief Default constructor for CostmapGenerator class
  /// \param [in] node_options A rclcpp::NodeOptions object passed on to rclcpp::Node
  explicit CostmapGenerator(const rclcpp::NodeOptions & node_options);

private:
  /// \brief Flag deciding whether use only wayarea or combined information
  /// when generating costmap layers
  bool use_wayarea_;

  /// \brief Costmap frame name
  std::string costmap_frame_;

  /// \brief Vehicle frame name
  std::string vehicle_frame_;

  /// \brief Map frame name
  std::string map_frame_;

  /// \brief The minimum value of costmap cell
  double grid_min_value_;

  /// \brief The maximum value of costmap cell
  double grid_max_value_;

  /// \brief Costmap resolution [m/cell]
  double grid_resolution_;

  /// \brief Costmap length in x direction [m]
  double grid_length_x_;

  /// \brief Costmap length in y direction [m]
  double grid_length_y_;

  /// \brief Position of the grid map in the grid map frame in x direction [m]
  double grid_position_x_;

  /// \brief Position of the grid map in the grid map frame in y direction [m]
  double grid_position_y_;

  /// \brief Padding which inflates requested HAD Map area, regarding start/goal pose bounding box [m]
  double route_box_padding_;

  /// \brief Costmap generated from fusion of HAD Map (and in future obstacles)
  grid_map::GridMap costmap_;

  /// \brief Debug publisher for created costmap visualisation
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_occupancy_grid_publisher_;

  /// \brief Debug publisher for received HAD Map
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_local_had_map_publisher_;

  /// \brief Server handling costmap generation requests
  rclcpp_action::Server<autoware_auto_msgs::action::PlannerCostmap>::SharedPtr costmap_action_server_;

  /// \brief Costmap generation request handle, used to end action inside HAD map request
  std::shared_ptr<rclcpp_action::ServerGoalHandle<autoware_auto_msgs::action::PlannerCostmap>> goal_handle_{nullptr};

  /// \brief HAD Map server client which requests most recent map after receiving costmap generation request
  rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedPtr map_client_;

  /// \brief TF buffer used to align costmap and vehicle position
  tf2_ros::Buffer tf_buffer_;

  /// \brief TF transform listener
  tf2_ros::TransformListener tf_listener_;

  /// \brief Temporary object handling lanelet areas
  std::vector<std::vector<geometry_msgs::msg::Point>> area_points_;

  /// \brief Struct with predefined costmap layer names
  struct LayerName
  {
    static constexpr const char * WAYAREA = "wayarea";
    static constexpr const char * COMBINED = "combined";
  };

  // action
  /// \brief Handle goal request from costmap generation client
  /// \return Server's response to goal request
  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const autoware_auto_msgs::action::PlannerCostmap::Goal>);

  /// \brief Handle costmap generation cancelation
  /// \return Action cancellation response
  rclcpp_action::CancelResponse handleCancel(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<autoware_auto_msgs::action::PlannerCostmap>>);

  /// \brief Handle actions after goal acceptance
  /// \param [in] goal_handle Costmap generation handle to be able to end costmap generation action
  void handleAccepted(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<autoware_auto_msgs::action::PlannerCostmap>>);

  /// \brief Align costmap and vehicle position in costmap frame
  void setCostmapPositionAtVehiclePosition();

  /// \brief Calculate bounding box of lanelet area points.
  /// \return Tuple containing points with maximum and minimum values
  ///         defining bounding box, regarding x and y coordinates
  std::tuple<grid_map::Position, grid_map::Position> calculateAreaPointsBoundingBox() const;

  /// \brief Reduce costmap size to sufficient minimum area defined by lanelet2 ways and areas
  grid_map::GridMap boundCostmap(const grid_map::Position &, const grid_map::Position &) const;

  /// \brief Create HAD map request for driveable area with bounds that are based on route
  /// \param [in] route Message with start, goal position and HAD map id's which define requested map area
  /// \return HAD map service request
  autoware_auto_msgs::srv::HADMapService::Request
  createMapRequest(const autoware_auto_msgs::msg::HADMapRoute &) const;

  /// \brief Callback for HAD map that composes costmap and successes action
  /// \param [in] future Enables receiving HAD map service response
  void mapResponse(rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedFuture future);

  /// \brief Initialize gridmap parameters based on rosparam
  void initGridmap();

  /// \brief Create occupancy grid based on input costmap (grid map) and parameters
  /// \param [in] costmap Grid map costmap implementation
  /// \return Costmap converted into ROS2 specific message
  nav_msgs::msg::OccupancyGrid createOccupancyGrid(const grid_map::GridMap &) const;

  /// \brief Set area_points_ from lanelet2 road area polygons
  /// \param [in] lanelet_map Lanelet2 map, received from HAD map provider
  void loadRoadAreasFromLaneletMap(lanelet::LaneletMapPtr);

  /// \brief Set area_points_ from lanelet2 parking areas
  /// \param [in] lanelet_map Lanelet2 map, received from HAD map provider
  void loadParkingAreasFromLaneletMap(lanelet::LaneletMapPtr);

  /// \brief Calculate costmap layer costs from lanelet2 map elements
  /// \return Costmap layer
  grid_map::Matrix generateWayAreaCostmap() const;

  /// \brief Calculate costmap layer costs for final output
  /// \return Costmap layer
  grid_map::Matrix generateCombinedCostmap() const;

  /// \brief Generate all costmap layers.
  void generateCostmapLayers();

  /// \brief Publish visualization of received part of the HAD map
  /// \param [in] map Lanelet2 map
  void publishLaneletVisualization(std::shared_ptr<lanelet::LaneletMap> & map);

  /// \brief Current state of costmap generator node and service
  CostmapGeneratorState state_;

  /// \brief Check if node can accept costmap generation request
  /// \return If node is in idle state
  bool isIdle() const { return state_ == CostmapGeneratorState::IDLE; }

  /// \brief Check if node is currently generating costmap
  /// \return Information if node is already generating costmap
  bool isGenerating() const { return state_ == CostmapGeneratorState::GENERATING; }

  /// \brief Nodes Idle state setter
  void setIdleState() { state_ = CostmapGeneratorState::IDLE; }

  /// \brief Nodes Generating state setter
  void setGeneratingState() { state_ = CostmapGeneratorState::GENERATING; }
};

}  // namespace autoware
}  // namespace planning
}  // namespace costmap_generator

#endif  // COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
