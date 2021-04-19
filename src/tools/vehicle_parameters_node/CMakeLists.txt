# Copyright 2021 the Autoware Foundation
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
project(vehicle_parameters_node)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(VEHICLE_PARAMETERS_NODE_LIB_SRC
  src/vehicle_parameters_node.cpp
)

set(VEHICLE_PARAMETERS_NODE_LIB_HEADERS
  include/vehicle_parameters_node/vehicle_parameters_node.hpp
)

# generate library
ament_auto_add_library(vehicle_parameters_node SHARED
  ${VEHICLE_PARAMETERS_NODE_LIB_SRC}
  ${VEHICLE_PARAMETERS_NODE_LIB_HEADERS}
)
autoware_set_compile_options(vehicle_parameters_node)

rclcpp_components_register_node(vehicle_parameters_node
  PLUGIN "autoware::vehicle_parameters_node::VehicleParametersNode"
  EXECUTABLE vehicle_parameters_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament package generation and installing
ament_auto_package()
