# Copyright 2021, The Autoware Foundation
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

project(interactive_trajectory_spoofer)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# generate library
set(INTERACTIVE_TRAJECTORY_SPOOFER_LIB "interactive_trajectory_spoofer")
ament_auto_add_library(${INTERACTIVE_TRAJECTORY_SPOOFER_LIB} SHARED
  src/interactive_trajectory_spoofer.cpp
  src/bezier.cpp
  include/interactive_trajectory_spoofer/interactive_trajectory_spoofer.hpp
  include/interactive_trajectory_spoofer/bezier.hpp
)
autoware_set_compile_options(${INTERACTIVE_TRAJECTORY_SPOOFER_LIB})
target_compile_options(${INTERACTIVE_TRAJECTORY_SPOOFER_LIB} PRIVATE)

# generate library
set(INTERACTIVE_TRAJECTORY_SPOOFER_NODE_LIB "interactive_trajectory_spoofer_node")
ament_auto_add_library(${INTERACTIVE_TRAJECTORY_SPOOFER_NODE_LIB} SHARED
  src/interactive_trajectory_spoofer_node.cpp
  include/interactive_trajectory_spoofer/interactive_trajectory_spoofer_node.hpp
)
target_link_libraries(${INTERACTIVE_TRAJECTORY_SPOOFER_NODE_LIB} ${INTERACTIVE_TRAJECTORY_SPOOFER_LIB})
autoware_set_compile_options(${INTERACTIVE_TRAJECTORY_SPOOFER_NODE_LIB})

rclcpp_components_register_node(${INTERACTIVE_TRAJECTORY_SPOOFER_NODE_LIB}
  PLUGIN "autoware::interactive_trajectory_spoofer::InteractiveTrajectorySpooferNode"
  EXECUTABLE ${INTERACTIVE_TRAJECTORY_SPOOFER_NODE_LIB}_exe
)

# Testing
if(BUILD_TESTING)
  # Static code analyzers
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_ros_test(
    test/interactive_trajectory_spoofer_launch.test.py
    TIMEOUT "30"
  )

  # Unit tests
  set(TEST_SOURCES test/test_interactive_trajectory_spoofer.cpp)
  set(TEST_INTERACTIVE_TRAJECTORY_SPOOFER_EXE test_trajectory_spoofer)
  ament_add_gtest(${TEST_INTERACTIVE_TRAJECTORY_SPOOFER_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_INTERACTIVE_TRAJECTORY_SPOOFER_EXE})
  target_compile_options(${TEST_INTERACTIVE_TRAJECTORY_SPOOFER_EXE} PRIVATE)
  ament_target_dependencies(${TEST_INTERACTIVE_TRAJECTORY_SPOOFER_EXE}
    "autoware_auto_common"
    "autoware_auto_msgs"
  )
  target_link_libraries(
    ${TEST_INTERACTIVE_TRAJECTORY_SPOOFER_EXE}
    ${INTERACTIVE_TRAJECTORY_SPOOFER_LIB}
    ${INTERACTIVE_TRAJECTORY_SPOOFER_NODE_LIB})
  target_include_directories(${TEST_INTERACTIVE_TRAJECTORY_SPOOFER_EXE} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

endif()

# Ament Exporting
ament_auto_package(INSTALL_TO_SHARE launch)
