# Copyright 2021 Arm Limited and Contributors.
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
project(monitored_node)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# list sources
set(MONITORED_NODE_LIB_HEADERS
  "include/${PROJECT_NAME}/monitored_node.hpp"
  "include/${PROJECT_NAME}/monitored_publisher.hpp"
  "include/${PROJECT_NAME}/monitored_subscription.hpp"
  "include/${PROJECT_NAME}/safety_monitor_interface.hpp"
  "include/${PROJECT_NAME}/visibility_control.hpp"
)

# Build the library
ament_auto_add_library(${PROJECT_NAME} SHARED
                        ${MONITORED_NODE_LIB_HEADERS})
autoware_set_compile_options(${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

if(BUILD_TESTING)
  # Enable code linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # compile each folder inside test/ as a test case
  find_package(ament_cmake_gtest REQUIRED)

  file(GLOB TEST_CASES test/*)
  foreach(TEST_FOLDER ${TEST_CASES})
    # the folder name becomes the test case name
    file(RELATIVE_PATH TEST_CASE_NAME
          ${CMAKE_CURRENT_LIST_DIR}/test ${TEST_FOLDER})
    # add all cpp files in the folder to the target
    file(GLOB TEST_CASE_SOURCES ${TEST_FOLDER}/*.cpp)
    ament_add_gtest(${TEST_CASE_NAME} ${TEST_CASE_SOURCES})
    target_link_libraries(${TEST_CASE_NAME} ${PROJECT_NAME})
  endforeach()

endif()

ament_auto_package()
