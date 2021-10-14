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
 *  * Redistributions of source code must retain the above copyright notice, private_node
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    private_node list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    private_node software without specific prior written permission.
 *
 *  private_node SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF private_node SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************/

#include "costmap_generator/costmap_generator.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "tf2/utils.h"

#include "had_map_utils/had_map_conversion.hpp"
#include "had_map_utils/had_map_query.hpp"
#include "had_map_utils/had_map_visualization.hpp"

#include "costmap_generator/object_map_utils.hpp"


// Convert from Point32 to Point
std::vector<geometry_msgs::msg::Point> poly2vector(const geometry_msgs::msg::Polygon & poly)
{
  std::vector<geometry_msgs::msg::Point> ps;
  for (const auto & p32 : poly.points) {
    geometry_msgs::msg::Point p;
    p.x = p32.x;
    p.y = p32.y;
    p.z = p32.z;
    ps.push_back(p);
  }
  return ps;
}

// taken from mapping/had_map/lanelet2_map_provider/src/lanelet2_map_visualizer.cpp
// TODO: make a common function out of this?
void insertMarkerArray(visualization_msgs::msg::MarkerArray & a1, const visualization_msgs::msg::MarkerArray & a2) {
  if (!a2.markers.empty()) {
    a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
  }
}


namespace autoware
{
namespace planning
{
namespace costmap_generator
{

CostmapGenerator::CostmapGenerator(const rclcpp::NodeOptions & node_options)
: Node("costmap_generator", node_options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  // Parameters
  costmap_frame_ = this->declare_parameter<std::string>("costmap_frame", "map");
  vehicle_frame_ = this->declare_parameter<std::string>("vehicle_frame", "base_link");
  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  grid_min_value_ = this->declare_parameter<double>("grid_min_value", 0.0);
  grid_max_value_ = this->declare_parameter<double>("grid_max_value", 1.0);
  grid_resolution_ = this->declare_parameter<double>("grid_resolution", 0.2);
  grid_length_x_ = this->declare_parameter<double>("grid_length_x", 50);
  grid_length_y_ = this->declare_parameter<double>("grid_length_y", 50);
  grid_position_x_ = this->declare_parameter<double>("grid_position_x", 0);
  grid_position_y_ = this->declare_parameter<double>("grid_position_y", 0);
  use_wayarea_ = this->declare_parameter<bool>("use_wayarea", true);
  route_box_padding_ = this->declare_parameter<double>("route_box_padding_m", 10.0);

  // Wait for first tf
  // We want to do this before creating subscriptions
  while (rclcpp::ok()) {
    try {
      tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      break;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Waiting for initial pose...");
    }
    rclcpp::sleep_for(std::chrono::seconds(5));
  }

  // Setup Map Service
  map_client_ = this->create_client<autoware_auto_msgs::srv::HADMapService>("~/client/HAD_Map_Service");

  while (!map_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for HAD map server.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for HAD map service...");
  }

  // Publishers
  debug_occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "~/debug/occupancy_grid", 1);
  debug_local_had_map_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/debug/local_had_map", 1);

  // Action server
  costmap_action_server_ = rclcpp_action::create_server<autoware_auto_msgs::action::PlannerCostmap>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "generate_costmap",
      [this](auto uuid, auto goal) {return this->handleGoal(uuid, goal);},
      [this](auto goal_handle) {return this->handleCancel(goal_handle);},
      [this](auto goal_handle) {return this->handleAccepted(goal_handle);});

  // Initialize
  initGridmap();
}

rclcpp_action::GoalResponse CostmapGenerator::handleGoal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const autoware_auto_msgs::action::PlannerCostmap::Goal>)
{
  if (!isIdle()) {
    RCLCPP_WARN(get_logger(), "Costmap generator is not in idle. Rejecting new request.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Received new costmap request.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CostmapGenerator::handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<autoware_auto_msgs::action::PlannerCostmap>>)
{
  RCLCPP_WARN(get_logger(), "Received action cancellation, rejecting.");
  return rclcpp_action::CancelResponse::REJECT;
}

void CostmapGenerator::handleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<autoware_auto_msgs::action::PlannerCostmap>> goal_handle)
{
  setGeneratingState();

  goal_handle_ = goal_handle;

  auto map_request = std::make_shared<autoware_auto_msgs::srv::HADMapService::Request>(
      createMapRequest(goal_handle_->get_goal()->route));

  // TODO: If synchronized service request is available, replace it with synchronized implementation
  auto result = map_client_->async_send_request(
      map_request,
      std::bind(&CostmapGenerator::mapResponse, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Requested HAD map.");
}

void CostmapGenerator::loadRoadAreasFromLaneletMap(const lanelet::LaneletMapPtr lanelet_map)
{
  auto road_lanelets = autoware::common::had_map_utils::getConstLaneletLayer(lanelet_map);

  for (const auto & road_lanelet : road_lanelets)
  {
    auto road_poly = autoware::common::had_map_utils::lanelet2Polygon(road_lanelet);
    area_points_.push_back(poly2vector(road_poly));
  }
}

void CostmapGenerator::loadParkingAreasFromLaneletMap(const lanelet::LaneletMapPtr lanelet_map)
{
  // Extract parking areas
  auto areas = autoware::common::had_map_utils::getAreaLayer(lanelet_map);
  auto parking_spot_areas = autoware::common::had_map_utils::subtypeAreas(areas, "parking_spot");
  auto parking_access_areas = autoware::common::had_map_utils::subtypeAreas(
      areas,
      "parking_access");

  // Collect points from parking spots
  for (const auto & parking_spot_area : parking_spot_areas)
  {
    auto parking_spot_poly = autoware::common::had_map_utils::area2Polygon(parking_spot_area);
    area_points_.push_back(poly2vector(parking_spot_poly));
  }

  // Collect points from parking spaces
  for (const auto & parking_access_area : parking_access_areas)
  {
    auto parking_access_poly = autoware::common::had_map_utils::area2Polygon(parking_access_area);
    area_points_.push_back(poly2vector(parking_access_poly));
  }
}

autoware_auto_msgs::srv::HADMapService::Request
CostmapGenerator::createMapRequest(const autoware_auto_msgs::msg::HADMapRoute & route) const
{
  auto request = autoware_auto_msgs::srv::HADMapService::Request();

  request.requested_primitives.push_back(
      autoware_auto_msgs::srv::HADMapService_Request::DRIVEABLE_GEOMETRY);

  // x
  request.geom_upper_bound.push_back(
      std::fmax(
          route.start_point.position.x,
          route.goal_point.position.x) + route_box_padding_);
  // y
  request.geom_upper_bound.push_back(
      std::fmax(
          route.start_point.position.y,
          route.goal_point.position.y) + route_box_padding_);
  // z (ignored)
  request.geom_upper_bound.push_back(0.0);

  // x
  request.geom_lower_bound.push_back(
      std::fmin(
          route.start_point.position.x,
          route.goal_point.position.x) - route_box_padding_);
  // y
  request.geom_lower_bound.push_back(
      std::fmin(
          route.start_point.position.y,
          route.goal_point.position.y) - route_box_padding_);
  // z (ignored)
  request.geom_lower_bound.push_back(0.0);

  return request;
}

void CostmapGenerator::setCostmapPositionAtVehiclePosition()
{
  // Get current pose
  geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(
      costmap_frame_, vehicle_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));

  // Set grid center
  grid_map::Position p;
  p.x() = tf.transform.translation.x;
  p.y() = tf.transform.translation.y;
  costmap_.setPosition(p);
}

std::tuple<grid_map::Position, grid_map::Position> CostmapGenerator::calculateAreaPointsBoundingBox() const
{
  grid_map::Position had_map_min_point = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
  grid_map::Position had_map_max_point = {std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};

  for (const auto &area : area_points_) {
    for (const auto &point : area) {
      had_map_max_point.x() = std::max(had_map_max_point.x(), point.x);
      had_map_max_point.y() = std::max(had_map_max_point.y(), point.y);
      had_map_min_point.x() = std::min(had_map_min_point.x(), point.x);
      had_map_min_point.y() = std::min(had_map_min_point.y(), point.y);
    }
  }

  return {had_map_min_point, had_map_max_point};
}

grid_map::GridMap
CostmapGenerator::boundCostmap(const grid_map::Position &min_point, const grid_map::Position &max_point) const {
  auto success = false;
  auto length = grid_map::Length(max_point.x() - min_point.x(),
                                 max_point.y() - min_point.y());
  auto position = grid_map::Position((max_point.x() + min_point.x()) / 2,
                                     (max_point.y() + min_point.y()) / 2);

  auto subcostmap = costmap_.getSubmap(position, length, success);

  if (!success) {
    RCLCPP_WARN(get_logger(), "Extracting submap failed, proceeding with whole costmap.");
    return costmap_;
  }

  return subcostmap;
}

void CostmapGenerator::generateCostmapLayers()
{
  if (use_wayarea_) {
    costmap_[LayerName::WAYAREA] = generateWayAreaCostmap();
  }

  costmap_[LayerName::COMBINED] = generateCombinedCostmap();
}

void CostmapGenerator::mapResponse(rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedFuture future)
{
  RCLCPP_INFO(get_logger(), "Received HAD map.");

  auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  autoware::common::had_map_utils::fromBinaryMsg(future.get()->map, lanelet_map_ptr);

  area_points_.clear();
  loadRoadAreasFromLaneletMap(lanelet_map_ptr);
  loadParkingAreasFromLaneletMap(lanelet_map_ptr);

  grid_map::Position had_map_min_point, had_map_max_point;
  std::tie(had_map_min_point, had_map_max_point) = calculateAreaPointsBoundingBox();

  try {
    setCostmapPositionAtVehiclePosition();
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    return;
  }

  generateCostmapLayers();

  auto costmap = boundCostmap(had_map_min_point, had_map_max_point);

  // Create result
  auto result = std::make_shared<autoware_auto_msgs::action::PlannerCostmap::Result>();
  auto out_occupancy_grid = createOccupancyGrid(costmap);
  result->costmap = out_occupancy_grid;

  // Publish visualizations
  publishLaneletVisualization(lanelet_map_ptr);
  debug_occupancy_grid_publisher_->publish(out_occupancy_grid);

  goal_handle_->succeed(result);

  RCLCPP_INFO(get_logger(), "Costmap generation succeeded.");

  setIdleState();
}

nav_msgs::msg::OccupancyGrid CostmapGenerator::createOccupancyGrid(const grid_map::GridMap & costmap) const
{
  nav_msgs::msg::OccupancyGrid occupancy_grid;

  // Convert to OccupancyGrid
  grid_map::GridMapRosConverter::toOccupancyGrid(
      costmap, LayerName::COMBINED, static_cast<float>(grid_min_value_), static_cast<float>(grid_max_value_), occupancy_grid);

  // Set header
  std_msgs::msg::Header header;
  header.frame_id = costmap_frame_;
  header.stamp = this->now();
  occupancy_grid.header = header;

  return occupancy_grid;
}

void CostmapGenerator::initGridmap()
{
  costmap_.setFrameId(costmap_frame_);
  costmap_.setGeometry(
    grid_map::Length(grid_length_x_, grid_length_y_), grid_resolution_,
    grid_map::Position(grid_position_x_, grid_position_y_));

  costmap_.add(LayerName::WAYAREA, grid_min_value_);
  costmap_.add(LayerName::COMBINED, grid_min_value_);
}

grid_map::Matrix CostmapGenerator::generateWayAreaCostmap() const
{
  grid_map::GridMap lanelet2_costmap = costmap_;
  if (!area_points_.empty()) {
    object_map::fillPolygonAreas(
        lanelet2_costmap, area_points_, LayerName::WAYAREA, static_cast<int>(grid_max_value_),  static_cast<int>(grid_min_value_),
         static_cast<int>(grid_min_value_),  static_cast<int>(grid_max_value_), costmap_frame_, map_frame_, tf_buffer_);
  }
  return lanelet2_costmap[LayerName::WAYAREA];
}

grid_map::Matrix CostmapGenerator::generateCombinedCostmap() const
{
  // assuming combined_costmap is calculated by element wise max operation
  grid_map::GridMap combined_costmap = costmap_;

  combined_costmap[LayerName::COMBINED].setConstant(static_cast<float>(grid_min_value_));

  combined_costmap[LayerName::COMBINED] =
    combined_costmap[LayerName::COMBINED].cwiseMax(combined_costmap[LayerName::WAYAREA]);

  return combined_costmap[LayerName::COMBINED];
}

// taken from mapping/had_map/lanelet2_map_provider/src/lanelet2_map_visualizer.cpp
// TODO: make a common function out of this?
void CostmapGenerator::publishLaneletVisualization(std::shared_ptr<lanelet::LaneletMap> & map) {
  auto lls = autoware::common::had_map_utils::getConstLaneletLayer(map);

  std_msgs::msg::ColorRGBA color_lane_bounds, color_parking_bounds,
  color_parking_access_bounds, color_geom_bounds,
  color_lanelets, color_parking, color_parking_access,
  color_pickup_dropoff;
  autoware::common::had_map_utils::setColor(
      &color_lane_bounds, 1.0f, 1.0f, 1.0f, 1.0f);
  autoware::common::had_map_utils::setColor(
      &color_parking_bounds, 1.0f, 1.0f, 1.0f, 1.0f);
  autoware::common::had_map_utils::setColor(
      &color_parking_access_bounds, 1.0f, 1.0f, 1.0f, 1.0f);
  autoware::common::had_map_utils::setColor(
      &color_geom_bounds, 0.0f, 0.0f, 1.0f, 1.0f);
  autoware::common::had_map_utils::setColor(
      &color_lanelets, 0.2f, 0.5f, 0.6f, 0.6f);
  autoware::common::had_map_utils::setColor(
      &color_parking, 0.3f, 0.3f, 0.7f, 0.5f);
  autoware::common::had_map_utils::setColor(
      &color_parking_access, 0.3f, 0.7f, 0.3f, 0.5f);
  autoware::common::had_map_utils::setColor(
      &color_pickup_dropoff, 0.9f, 0.2f, 0.1f, 0.7f);

  visualization_msgs::msg::MarkerArray map_marker_array;

  rclcpp::Time marker_t = rclcpp::Time(0);
  insertMarkerArray(
      map_marker_array,
      autoware::common::had_map_utils::laneletsBoundaryAsMarkerArray(
          marker_t, lls,
          color_lane_bounds, true));
  insertMarkerArray(
      map_marker_array,
      autoware::common::had_map_utils::laneletsAsTriangleMarkerArray(
          marker_t,
          "lanelet_triangles", lls, color_lanelets));

  // for parking spots defined as areas (LaneletOSM definition)
  auto ll_areas = autoware::common::had_map_utils::getAreaLayer(map);
  auto ll_parking_areas = autoware::common::had_map_utils::subtypeAreas(ll_areas, "parking_spot");
  auto ll_parking_access_areas = autoware::common::had_map_utils::subtypeAreas(
      ll_areas,
      "parking_access");

  insertMarkerArray(
      map_marker_array,
      autoware::common::had_map_utils::areasBoundaryAsMarkerArray(
          marker_t, "parking_area_bounds",
          ll_parking_areas, color_parking_bounds));
  insertMarkerArray(
      map_marker_array,
      autoware::common::had_map_utils::areasBoundaryAsMarkerArray(
          marker_t, "parking_access_area_bounds",
          ll_parking_access_areas, color_parking_bounds));
  insertMarkerArray(
      map_marker_array,
      autoware::common::had_map_utils::areasAsTriangleMarkerArray(
          marker_t, "parking_area_triangles",
          ll_parking_areas, color_parking));
  insertMarkerArray(
      map_marker_array,
      autoware::common::had_map_utils::areasAsTriangleMarkerArray(
          marker_t, "parking_access_area_triangles",
          ll_parking_access_areas, color_parking_access));

  debug_local_had_map_publisher_->publish(map_marker_array);
}

}  // namespace autoware
}  // namespace planning
}  // namespace costmap_generator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::planning::costmap_generator::CostmapGenerator)