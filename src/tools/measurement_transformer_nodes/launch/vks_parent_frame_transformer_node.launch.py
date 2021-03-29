# Copyright 2021, the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch file for VehicleKinematicStateParentTransformer."""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Generate launch description for VehicleKinematicStateParentTransformer."""
    pkg_prefix = get_package_share_directory('measurement_transformer_nodes')
    vehicle_kinematic_state_parent_frame_transformer_param_file = os.path.join(
        pkg_prefix,
        'param/vehicle_kinematic_state_parent_frame_transformer.param.yaml'
    )

    vehicle_kinematic_state_parent_frame_transformer_param = DeclareLaunchArgument(
        'vehicle_kinematic_state_parent_frame_transformer_param_file',
        default_value=vehicle_kinematic_state_parent_frame_transformer_param_file,
        description='Path to config file for VehicleKinematicStateParentTransformerNode'
    )

    vehicle_kinematic_state_parent_frame_transformer_node = Node(
        name='vehicle_kinematic_state_parent_frame_transformer_node',
        namespace='',
        package='measurement_transformer_nodes',
        executable='vehicle_kinematic_state_parent_frame_transformer_node_exe',
        parameters=[
          LaunchConfiguration('vehicle_kinematic_state_parent_frame_transformer_param_file')
        ],
        remappings=[
            ("vks_out", "/vehicle/vehicle_kinematic_state"),
            ("vks_in", "converted_data")
        ],
        output='screen',
    )

    return LaunchDescription([
        vehicle_kinematic_state_parent_frame_transformer_param,
        vehicle_kinematic_state_parent_frame_transformer_node,
    ])
