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

project(measurement_transformer_nodes)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

# generate nodes library
set(MEASUREMENT_TRANSFORMER_NODES_SRC
  src/pose_child_frame_transformer_node.cpp
  src/pose_parent_frame_transformer_node.cpp
  src/pose_with_cov_child_frame_transformer_node.cpp
  src/pose_with_cov_parent_frame_transformer_node.cpp
  src/vehicle_kinematic_state_child_frame_transformer_node.cpp
  src/vehicle_kinematic_state_parent_frame_transformer_node.cpp
)

set(MEASUREMENT_TRANSFORMER_NODES_HEADERS
  include/measurement_transformer_nodes/measurement_transformer_node_base.hpp
  include/measurement_transformer_nodes/pose_child_frame_transformer_node.hpp
  include/measurement_transformer_nodes/pose_parent_frame_transformer_node.hpp
  include/measurement_transformer_nodes/pose_with_cov_child_frame_transformer_node.hpp
  include/measurement_transformer_nodes/pose_with_cov_parent_frame_transformer_node.hpp
  include/measurement_transformer_nodes/vehicle_kinematic_state_child_frame_transformer_node.hpp
  include/measurement_transformer_nodes/vehicle_kinematic_state_parent_frame_transformer_node.hpp
  include/measurement_transformer_nodes/visibility_control.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_nodes SHARED
  ${MEASUREMENT_TRANSFORMER_NODES_SRC}
  ${MEASUREMENT_TRANSFORMER_NODES_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME}_nodes)
target_compile_options(${PROJECT_NAME}_nodes PRIVATE -Wno-old-style-cast)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "autoware::${PROJECT_NAME}::PoseChildFrameTransformerNode"
  EXECUTABLE pose_child_frame_transformer_node_exe
)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "autoware::${PROJECT_NAME}::PoseParentFrameTransformerNode"
  EXECUTABLE pose_parent_frame_transformer_node_exe
)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "autoware::${PROJECT_NAME}::PoseWithCovChildFrameTransformerNode"
  EXECUTABLE pose_with_cov_child_frame_transformer_node_exe
)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "autoware::${PROJECT_NAME}::PoseWithCovParentFrameTransformerNode"
  EXECUTABLE pose_with_cov_parent_frame_transformer_node_exe
)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "autoware::${PROJECT_NAME}::VehicleKinematicStateChildFrameTransformerNode"
  EXECUTABLE vehicle_kinematic_state_child_frame_transformer_node_exe
)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "autoware::${PROJECT_NAME}::VehicleKinematicStateParentFrameTransformerNode"
  EXECUTABLE vehicle_kinematic_state_parent_frame_transformer_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
