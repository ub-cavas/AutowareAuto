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

project(multi_object_tracking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(MULTI_OBJECT_TRACKING_LIB_SRC
  src/multi_object_tracking.cpp
)

set(MULTI_OBJECT_TRACKING_LIB_HEADERS
  include/multi_object_tracking/multi_object_tracking.hpp
  include/multi_object_tracking/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${MULTI_OBJECT_TRACKING_LIB_SRC}
  ${MULTI_OBJECT_TRACKING_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_multi_object_tracking.cpp)
  set(TEST_MULTI_OBJECT_TRACKING_EXE test_multi_object_tracking)
  ament_add_gtest(${TEST_MULTI_OBJECT_TRACKING_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_MULTI_OBJECT_TRACKING_EXE})
  target_link_libraries(${TEST_MULTI_OBJECT_TRACKING_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package()
