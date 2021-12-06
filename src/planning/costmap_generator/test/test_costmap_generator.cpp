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

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>

#include "had_map_utils/had_map_query.hpp"
#include "costmap_generator/costmap_generator.hpp"

using autoware::planning::costmap_generator::CostmapGenerator;
using autoware::planning::costmap_generator::CostmapGeneratorParams;
using autoware::planning::costmap_generator::LayerName;

CostmapGeneratorParams generateExampleParameters(
  const bool bound_costmap = true, const bool use_wayarea = true)
{
  auto params = CostmapGeneratorParams();

  params.grid_min_value = 0.0;
  params.grid_max_value = 1.0;
  params.grid_resolution = 0.2;
  params.grid_length_x = 70.0;
  params.grid_length_y = 70.0;
  params.grid_position_x = 0.0;
  params.grid_position_y = 0.0;
  params.use_wayarea = use_wayarea;
  params.bound_costmap = bound_costmap;
  params.costmap_frame = "map";

  return params;
}

std::shared_ptr<lanelet::LaneletMap> createLanelet(
  const double & resolution, const double & width = 2.0,
  const double & length = 5.0)
{
  using lanelet::Point3d;
  using lanelet::LineString3d;
  using lanelet::Area;
  using lanelet::utils::getId;

  double padding = resolution / 2.0;
  // prepare parking place points
  Point3d p1(getId(), padding, padding, 0.0);
  Point3d p2(getId(), padding, width + padding, 0.0);
  Point3d p3(getId(), length + padding, width + padding, 0.0);
  Point3d p4(getId(), length + padding, padding, 0.0);

  // prepare linestrings
  LineString3d bottom(getId(), {p1, p2});
  LineString3d left(getId(), {p2, p3});
  LineString3d top(getId(), {p3, p4});
  LineString3d right(getId(), {p4, p1});

  Area area(getId(), {top, right, bottom, left});
  area.attributes()[lanelet::AttributeName::Subtype] = "parking_spot";

  return
    std::make_shared<lanelet::LaneletMap>(std::move(*lanelet::utils::createMap({}, {area})));
}

int countCostmapToLaneletMismatch(
  const grid_map::GridMap costmap, const std::shared_ptr<lanelet::LaneletMap> lanalet_map_ptr,
  const std::string & layer_name, const double & costmap_resolution, const float & treshold)
{
  // number of cells which are not pointing to lanelet parking place
  int counter = 0;

  const grid_map::Matrix & map = costmap[layer_name];
  for (grid_map::GridMapIterator iterator(costmap); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    auto cell_value = map(index(0), index(1));

    // if cell is not free space
    if (cell_value > treshold) {
      continue;
    }

    grid_map::Position position;
    costmap.getPosition(index, position);
    auto resolution_half = costmap_resolution / 2.0;

    // get grid map cell
    lanelet::BasicPoint2d point_min{
      position.x() - resolution_half,
      position.y() - resolution_half};
    lanelet::BasicPoint2d point_max{
      position.x() + resolution_half,
      position.y() + resolution_half};

    lanelet::BoundingBox2d box;
    box.min() = point_min;
    box.max() = point_max;

    // check parking place and costmap cell bounding boxes intersection
    const auto nearest_lanelets = lanalet_map_ptr->areaLayer.search(box);

    // if no intersection found add mismatch
    if (nearest_lanelets.empty()) {
      counter++;
    }
  }
  return counter;
}

TEST(CostmapGeneratorTest, GenerateValidCostmap)
{
  // initialize costmap generator
  auto costmap_param = generateExampleParameters();
  auto costmap_generator = std::make_unique<CostmapGenerator>(costmap_param);

  auto lanelet_map_ptr = createLanelet(costmap_param.grid_resolution);

  // grid is placed in the middle of map frame
  grid_map::Position vehicle_to_grid_position(0.0, 0.0);
  // map frame = costmap frame
  geometry_msgs::msg::TransformStamped costmap_to_map_transform;

  auto costmap = costmap_generator->generateCostmap(
    lanelet_map_ptr, vehicle_to_grid_position, costmap_to_map_transform);

  auto cell_value_treshold =
    static_cast<float>((costmap_param.grid_min_value + costmap_param.grid_max_value) / 2.0);

  // treshold for cells not specifically inside lanelet
  auto mismatch_treshold = 10;

  auto number_of_mismatches = countCostmapToLaneletMismatch(
    costmap, lanelet_map_ptr, LayerName::COMBINED,
    costmap_param.grid_resolution, cell_value_treshold);

  EXPECT_LT(number_of_mismatches, mismatch_treshold);
}

TEST(CostmapGeneratorTest, GenerateBoundedCostmap)
{
  // initialize costmap generator
  auto costmap_param = generateExampleParameters();
  auto costmap_generator = std::make_unique<CostmapGenerator>(costmap_param);

  auto lanelet_map_ptr = createLanelet(costmap_param.grid_resolution);

  // grid is placed in the middle of map frame
  grid_map::Position vehicle_to_grid_position(0.0, 0.0);
  // map frame = costmap frame
  geometry_msgs::msg::TransformStamped costmap_to_map_transform;

  auto costmap = costmap_generator->generateCostmap(
    lanelet_map_ptr, vehicle_to_grid_position, costmap_to_map_transform);

  EXPECT_LT(costmap.getLength().x(), costmap_param.grid_length_x);
  EXPECT_LT(costmap.getLength().y(), costmap_param.grid_length_y);
}

TEST(CostmapGeneratorTest, GenerateCostmapWithoutBounding)
{
  // initialize costmap generator
  auto costmap_param = generateExampleParameters(false);
  auto costmap_generator = std::make_unique<CostmapGenerator>(costmap_param);

  auto lanelet_map_ptr = createLanelet(costmap_param.grid_resolution);

  // grid is placed in the middle of map frame
  grid_map::Position vehicle_to_grid_position(0.0, 0.0);
  // map frame = costmap frame
  geometry_msgs::msg::TransformStamped costmap_to_map_transform;

  auto costmap = costmap_generator->generateCostmap(
    lanelet_map_ptr, vehicle_to_grid_position, costmap_to_map_transform);

  EXPECT_DOUBLE_EQ(costmap.getResolution(), costmap_param.grid_resolution);
  EXPECT_DOUBLE_EQ(costmap.getLength().x(), costmap_param.grid_length_x);
  EXPECT_DOUBLE_EQ(costmap.getLength().y(), costmap_param.grid_length_y);
}

TEST(CostmapGeneratorTest, GenerateCostmapWithoutWayareasExpectOnlyObstacles)
{
  // initialize costmap generator
  auto costmap_param = generateExampleParameters(false, false);
  auto costmap_generator = std::make_unique<CostmapGenerator>(costmap_param);

  auto lanelet_map_ptr = createLanelet(costmap_param.grid_resolution);

  // grid is placed in the middle of map frame
  grid_map::Position vehicle_to_grid_position(0.0, 0.0);
  // map frame = costmap frame
  geometry_msgs::msg::TransformStamped costmap_to_map_transform;

  auto costmap = costmap_generator->generateCostmap(
    lanelet_map_ptr, vehicle_to_grid_position, costmap_to_map_transform);

  const grid_map::Matrix & map = costmap[LayerName::COMBINED];
  for (grid_map::GridMapIterator iterator(costmap); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    const auto cell_value = static_cast<double>(map(index(0), index(1)));
    if (cell_value != costmap_param.grid_max_value) {
      ASSERT_TRUE(false);
    }
  }
}

TEST(CostmapGeneratorTest, InitializeCostmapWithZeroResolution)
{
  // initialize costmap generator
  auto costmap_params = generateExampleParameters();
  costmap_params.grid_resolution = 0.0;
  EXPECT_THROW(std::make_unique<CostmapGenerator>(costmap_params), std::invalid_argument);
}

TEST(CostmapGeneratorTest, InitializeCostmapWithZeroXLength)
{
  // initialize costmap generator
  auto costmap_params = generateExampleParameters();
  costmap_params.grid_length_x = 0.0;
  EXPECT_THROW(std::make_unique<CostmapGenerator>(costmap_params), std::invalid_argument);
}

TEST(CostmapGeneratorTest, InitializeCostmapWithZeroYLength)
{
  // initialize costmap generator
  auto costmap_params = generateExampleParameters();
  costmap_params.grid_length_y = 0.0;
  EXPECT_THROW(std::make_unique<CostmapGenerator>(costmap_params), std::invalid_argument);
}
