# Copyright 2021 the Autoware Foundation
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

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """
    Launch selected modules
    """

    autoware_auto_launch_pkg_prefix = get_package_share_directory(
        'autoware_auto_launch')

    # mapping
    with_mapping = DeclareLaunchArgument(
        'with_mapping',
        default_value='True',
        description='launch mapping module when true'
    )
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([autoware_auto_launch_pkg_prefix, '/launch/autoware_auto_mapping.launch.py']),
        launch_arguments={}.items(),
        condition = IfCondition(LaunchConfiguration('with_mapping'))
    )

    # localization
    with_localization = DeclareLaunchArgument(
        'with_localization',
        default_value='True',
        description='launch localization module when true'
    )
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([autoware_auto_launch_pkg_prefix, '/launch/autoware_auto_localization.launch.py']),
        launch_arguments={}.items(),
        condition = IfCondition(LaunchConfiguration('with_localization'))
    )

    # perception
    with_perception = DeclareLaunchArgument(
        'with_perception',
        default_value='True',
        description='launch perception module when true'
    )
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([autoware_auto_launch_pkg_prefix, '/launch/autoware_auto_perception.launch.py']),
        launch_arguments={}.items(),
        condition = IfCondition(LaunchConfiguration('with_perception'))
    )

    # planning
    with_planning = DeclareLaunchArgument(
        'with_planning',
        default_value='True',
        description='launch planning module when true'
    )
    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([autoware_auto_launch_pkg_prefix, '/launch/autoware_auto_planning.launch.py']),
        launch_arguments={}.items(),
        condition = IfCondition(LaunchConfiguration('with_planning'))
    )

    # sensors
    with_sensors = DeclareLaunchArgument(
        'with_sensors',
        default_value='True',
        description='launch sensors module when true'
    )
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([autoware_auto_launch_pkg_prefix, '/launch/autoware_auto_sensors.launch.py']),
        launch_arguments={}.items(),
        condition = IfCondition(LaunchConfiguration('with_sensors'))
    )

    # system
    with_system = DeclareLaunchArgument(
        'with_system',
        default_value='True',
        description='launch system module when true'
    )
    system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([autoware_auto_launch_pkg_prefix, '/launch/autoware_auto_system.launch.py']),
        launch_arguments={}.items(),
        condition = IfCondition(LaunchConfiguration('with_system'))
    )

    # visualization
    with_visualization = DeclareLaunchArgument(
        'with_visualization',
        default_value='True',
        description='launch visualization module when true'
    )
    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([autoware_auto_launch_pkg_prefix, '/launch/autoware_auto_visualization.launch.py']),
        launch_arguments={}.items(),
        condition = IfCondition(LaunchConfiguration('with_visualization'))
    )

    return LaunchDescription([
        with_mapping,
        with_localization,
        with_perception,
        with_planning,
        with_sensors,
        with_system,
        with_visualization,
        mapping_launch,
        localization_launch,
        perception_launch,
        planning_launch,
        sensors_launch,
        system_launch,
        visualization_launch,
    ])
