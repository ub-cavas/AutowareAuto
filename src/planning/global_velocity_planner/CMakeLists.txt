# Copyright 2021 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(global_velocity_planner)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(GLOBAL_VELOCITY_PLANNER_LIB_SRC
  src/global_velocity_planner.cpp
)

set(GLOBAL_VELOCITY_PLANNER_LIB_HEADERS
  include/global_velocity_planner/global_velocity_planner.hpp
  include/global_velocity_planner/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${GLOBAL_VELOCITY_PLANNER_LIB_SRC}
  ${GLOBAL_VELOCITY_PLANNER_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

set(GLOBAL_VELOCITY_PLANNER_NODE_SRC
  src/global_velocity_planner_node.cpp
)

set(GLOBAL_VELOCITY_PLANNER_NODE_HEADERS
  include/global_velocity_planner/global_velocity_planner_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${GLOBAL_VELOCITY_PLANNER_NODE_SRC}
  ${GLOBAL_VELOCITY_PLANNER_NODE_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME}_node)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::global_velocity_planner::GlobalVelocityPlannerNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_global_velocity_planner.cpp)
  set(TEST_GLOBAL_VELOCITY_PLANNER_EXE test_global_velocity_planner)
  ament_add_gtest(${TEST_GLOBAL_VELOCITY_PLANNER_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_GLOBAL_VELOCITY_PLANNER_EXE})
  target_link_libraries(${TEST_GLOBAL_VELOCITY_PLANNER_EXE} ${PROJECT_NAME})

  # Integration test

  add_ros_test(
          test/global_velocity_planner_node_launch.test.py
          TIMEOUT "30"
  )
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
