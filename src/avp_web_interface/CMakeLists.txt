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

project(avp_web_interface)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(AVP_WEB_INTERFACE_LIB_SRC
  src/avp_web_interface.cpp
)

set(AVP_WEB_INTERFACE_LIB_HEADERS
  include/avp_web_interface/avp_web_interface.hpp
  include/avp_web_interface/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${AVP_WEB_INTERFACE_LIB_SRC}
  ${AVP_WEB_INTERFACE_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

set(AVP_WEB_INTERFACE_NODE_SRC
  src/avp_web_interface_node.cpp
)

set(AVP_WEB_INTERFACE_NODE_HEADERS
  include/avp_web_interface/avp_web_interface_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${AVP_WEB_INTERFACE_NODE_SRC}
  ${AVP_WEB_INTERFACE_NODE_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME}_node)
rclcpp_components_register_nodes(${PROJECT_NAME}_node "autoware::avp_web_interface::AvpWebInterfaceNode")

set(AVP_WEB_INTERFACE_EXE_SRC
  src/main.cpp
)

# generate stand-alone executable
ament_auto_add_executable(${PROJECT_NAME}_exe
  ${AVP_WEB_INTERFACE_EXE_SRC}
)
autoware_set_compile_options(${PROJECT_NAME}_exe)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/gtest_main.cpp test/test_avp_web_interface.cpp)
  set(TEST_AVP_WEB_INTERFACE_EXE test_avp_web_interface)
  ament_add_gtest(${TEST_AVP_WEB_INTERFACE_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_AVP_WEB_INTERFACE_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package()
