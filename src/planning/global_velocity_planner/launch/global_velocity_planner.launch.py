# Copyright 2022 Leo Drive Teknoloji A.Ş.
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

"""Launch Modules for Milestone 3 of the AVP 2020 Demo."""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """
    Launch all nodes defined in the architecture for Milestone 3 of the AVP 2020 Demo.

    More details about what is included can
    be found at https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/milestones/25.
    """
    # avp_demo_pkg_prefix = get_package_share_directory('autoware_demos')
    # autoware_launch_pkg_prefix = get_package_share_directory('autoware_auto_launch')
    global_vel_planner_prefix = get_package_share_directory('global_velocity_planner')

    global_velocity_planner_param_file = os.path.join(
        global_vel_planner_prefix, 'param/defaults.param.yaml')

    global_velocity_planner_param = DeclareLaunchArgument(
        'global_velocity_planner_param_file',
        default_value=global_velocity_planner_param_file,
        description='Path to parameter file for global velocity planner'
    )
    # Nodes
    global_velocity_planner = Node(
        package='global_velocity_planner',
        name='global_velocity_planner_node',
        namespace='planning',
        executable='global_velocity_planner_node_exe',
        parameters=[
            LaunchConfiguration('global_velocity_planner_param_file'),
        ],
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
                    ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
                    ('route', 'global_path')]
    )

    return LaunchDescription([
        global_velocity_planner_param,
        global_velocity_planner,
    ])
