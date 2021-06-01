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

project(autoware_auto_msg_transformations)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(AUTOWARE_AUTO_MSG_TRANSFORMATIONS_LIB_SRC
  src/autoware_auto_msg_transformations.cpp
)

set(AUTOWARE_AUTO_MSG_TRANSFORMATIONS_LIB_HEADERS
  include/autoware_auto_msg_transformations/autoware_auto_msg_transformations.hpp
  include/autoware_auto_msg_transformations/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${AUTOWARE_AUTO_MSG_TRANSFORMATIONS_LIB_SRC}
  ${AUTOWARE_AUTO_MSG_TRANSFORMATIONS_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_autoware_auto_msg_transformations.cpp)
  set(TEST_AUTOWARE_AUTO_MSG_TRANSFORMATIONS_EXE test_autoware_auto_msg_transformations)
  ament_add_gtest(${TEST_AUTOWARE_AUTO_MSG_TRANSFORMATIONS_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_AUTOWARE_AUTO_MSG_TRANSFORMATIONS_EXE})
  target_link_libraries(${TEST_AUTOWARE_AUTO_MSG_TRANSFORMATIONS_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package()
