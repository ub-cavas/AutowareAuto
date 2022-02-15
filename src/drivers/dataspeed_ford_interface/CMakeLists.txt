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

project(dataspeed_ford_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(DATASPEED_FORD_INTERFACE_LIB_SRC
  src/dataspeed_ford_interface.cpp
)

set(DATASPEED_FORD_INTERFACE_LIB_HEADERS
  include/dataspeed_ford_interface/dataspeed_ford_interface.hpp
  include/dataspeed_ford_interface/visibility_control.hpp
  include/dataspeed_ford_interface/pid_controller.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${DATASPEED_FORD_INTERFACE_LIB_SRC}
  ${DATASPEED_FORD_INTERFACE_LIB_HEADERS}
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-old-style-cast)

set(DATASPEED_FORD_INTERFACE_NODE_SRC
  src/dataspeed_ford_interface_node.cpp
)

set(DATASPEED_FORD_INTERFACE_NODE_HEADERS
  include/dataspeed_ford_interface/dataspeed_ford_interface_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${DATASPEED_FORD_INTERFACE_NODE_SRC}
  ${DATASPEED_FORD_INTERFACE_NODE_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME}_node)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::dataspeed_ford_interface::DataspeedFordInterfaceNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_dataspeed_ford_interface.cpp)
  set(TEST_DATASPEED_FORD_INTERFACE_EXE test_dataspeed_ford_interface)
  ament_add_gtest(${TEST_DATASPEED_FORD_INTERFACE_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_DATASPEED_FORD_INTERFACE_EXE})
  target_link_libraries(${TEST_DATASPEED_FORD_INTERFACE_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
