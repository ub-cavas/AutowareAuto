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

project(autoware_utils)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost REQUIRED)

set(AUTOWARE_UTILS_SRC
  src/autoware_utils.cpp
  src/planning/planning_marker_helper.cpp)

set(AUTOWARE_UTILS_HEADERS
  include/autoware_utils/autoware_utils.hpp
  include/autoware_utils/geometry/boost_geometry.hpp
  include/autoware_utils/geometry/geometry.hpp
  include/autoware_utils/geometry/pose_deviation.hpp
  include/autoware_utils/math/constants.hpp
  include/autoware_utils/math/normalization.hpp
  include/autoware_utils/math/range.hpp
  include/autoware_utils/math/unit_conversion.hpp
  include/autoware_utils/planning/planning_marker_helper.hpp
  include/autoware_utils/ros/debug_publisher.hpp
  include/autoware_utils/ros/debug_traits.hpp
  include/autoware_utils/ros/marker_helper.hpp
  include/autoware_utils/ros/processing_time_publisher.hpp
  include/autoware_utils/ros/self_pose_listener.hpp
  include/autoware_utils/ros/transform_listener.hpp
  include/autoware_utils/ros/update_param.hpp
  include/autoware_utils/ros/wait_for_param.hpp
  include/autoware_utils/system/heartbeat_checker.hpp
  include/autoware_utils/system/stop_watch.hpp
  include/autoware_utils/trajectory/trajectory.hpp)

# generate component node library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${AUTOWARE_UTILS_SRC}
  ${AUTOWARE_UTILS_HEADERS})

autoware_set_compile_options(${PROJECT_NAME})

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-old-style-cast)

# Test
if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB_RECURSE test_files test/**/*.cpp)
  ament_add_gtest(test_autoware_utils ${test_files})
  autoware_set_compile_options(test_autoware_utils)
  target_compile_options(test_autoware_utils PRIVATE -Wno-old-style-cast)
  target_link_libraries(test_autoware_utils autoware_utils)
endif()

ament_auto_package()
