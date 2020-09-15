# Copyright 2020 The Autoware Foundation
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

project(ssc_interface)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(SSC_INTERFACE_LIB_SRC
  src/ssc_interface.cpp
)

set(SSC_INTERFACE_LIB_HEADERS
  include/ssc_interface/ssc_interface.hpp
  include/ssc_interface/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${SSC_INTERFACE_LIB_SRC}
  ${SSC_INTERFACE_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

set(SSC_INTERFACE_NODE_SRC
  src/ssc_interface_node.cpp
)

set(SSC_INTERFACE_NODE_HEADERS
  include/ssc_interface/ssc_interface_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${SSC_INTERFACE_NODE_SRC}
  ${SSC_INTERFACE_NODE_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME}_node)
rclcpp_components_register_node(${PROJECT_NAME}_node
  "autoware::ssc_interface::SscInterfaceNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_ssc_interface.cpp)
  set(TEST_SSC_INTERFACE_EXE test_ssc_interface)
  ament_add_gtest(${TEST_SSC_INTERFACE_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_SSC_INTERFACE_EXE} ${PROJECT_NAME})
  add_dependencies(${TEST_SSC_INTERFACE_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package()
