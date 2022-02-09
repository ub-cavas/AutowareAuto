# Copyright 2021 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(behavior_path_planner)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
find_package(interpolation REQUIRED)
find_package(BehaviorTreeV3)

set(BEHAVIOR_PATH_PLANNER_SRC
  src/behavior_path_planner_node.cpp
  src/behavior_tree_manager.cpp
  src/route_handler.cpp
  src/utilities.cpp
  src/path_utilities.cpp
  src/path_shifter/path_shifter.cpp
  src/turn_signal_decider.cpp
  src/scene_module/scene_module_bt_node_interface.cpp
  src/scene_module/side_shift/side_shift_module.cpp
  src/scene_module/side_shift/util.cpp
  src/scene_module/avoidance/avoidance_module.cpp
  src/scene_module/avoidance/avoidance_utils.cpp
  src/scene_module/avoidance/debug.cpp
  src/scene_module/lane_following/lane_following_module.cpp
  src/scene_module/lane_change/lane_change_module.cpp
  src/scene_module/lane_change/util.cpp
  src/scene_module/pull_over/pull_over_module.cpp
  src/scene_module/pull_over/util.cpp
  src/scene_module/pull_out/pull_out_module.cpp
  src/scene_module/pull_out/util.cpp)

set(BEHAVIOR_PATH_PLANNER_HEADERS
  include/behavior_path_planner/behavior_path_planner_node.hpp
  include/behavior_path_planner/behavior_tree_manager.hpp
  include/behavior_path_planner/data_manager.hpp
  include/behavior_path_planner/parameters.hpp
  include/behavior_path_planner/path_utilities.hpp
  include/behavior_path_planner/predicted_objects_msg.hpp
  include/behavior_path_planner/route_handler.hpp
  include/behavior_path_planner/turn_signal_decider.hpp
  include/behavior_path_planner/utilities.hpp
  include/behavior_path_planner/path_shifter/path_shifter.hpp
  include/behavior_path_planner/util/create_vehicle_footprint.hpp
  include/behavior_path_planner/scene_module/approval_handler.hpp
  include/behavior_path_planner/scene_module/scene_module_bt_node_interface.hpp
  include/behavior_path_planner/scene_module/scene_module_interface.hpp
  include/behavior_path_planner/scene_module/avoidance/avoidance_module.hpp
  include/behavior_path_planner/scene_module/avoidance/avoidance_module_data.hpp
  include/behavior_path_planner/scene_module/avoidance/avoidance_utils.hpp
  include/behavior_path_planner/scene_module/avoidance/debug.hpp
  include/behavior_path_planner/scene_module/lane_change/lane_change_module.hpp
  include/behavior_path_planner/scene_module/lane_change/util.hpp
  include/behavior_path_planner/scene_module/lane_following/lane_following_module.hpp
  include/behavior_path_planner/scene_module/pull_out/pull_out_module.hpp
  include/behavior_path_planner/scene_module/pull_out/util.hpp
  include/behavior_path_planner/scene_module/pull_over/pull_over_module.hpp
  include/behavior_path_planner/scene_module/pull_over/util.hpp
  include/behavior_path_planner/scene_module/side_shift/side_shift_module.hpp
  include/behavior_path_planner/scene_module/side_shift/util.hpp)

# generate component node library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${BEHAVIOR_PATH_PLANNER_SRC}
  ${BEHAVIOR_PATH_PLANNER_HEADERS})

autoware_set_compile_options(${PROJECT_NAME})

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-old-style-cast)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "behavior_path_planner::BehaviorPathPlannerNode"
  EXECUTABLE ${PROJECT_NAME}_exe)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  BT::behaviortree_cpp_v3)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_behavior_path_planner_node.cpp)
  set(TEST_BEHAVIOR_PATH_PLANNER_NODE_EXE test_behavior_path_planner_node)
  ament_add_gtest(${TEST_BEHAVIOR_PATH_PLANNER_NODE_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_BEHAVIOR_PATH_PLANNER_NODE_EXE})
  target_compile_options(${TEST_BEHAVIOR_PATH_PLANNER_NODE_EXE} PRIVATE
    -Wno-old-style-cast)
  target_link_libraries(${TEST_BEHAVIOR_PATH_PLANNER_NODE_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  config
  launch)