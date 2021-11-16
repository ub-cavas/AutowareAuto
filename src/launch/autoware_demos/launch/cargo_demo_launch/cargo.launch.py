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
#
# Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

"""Launch required subsystems for running scenario simulator."""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution

import os
from enum import Enum

def launch_setup(context, *args, **kwargs):
    autoware_launch_prefix = get_package_share_directory('autoware_demos')

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([autoware_launch_prefix,
                                       '/launch/cargo_demo_launch/cargo_mapping.launch.py']),
        launch_arguments={}.items()
    )

    vehicle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([autoware_launch_prefix,
                                       '/launch/cargo_demo_launch/cargo_vehicle.launch.py']),
        launch_arguments={}.items()
    )

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([autoware_launch_prefix,
                                       '/launch/cargo_demo_launch/cargo_sensors.launch.py']),
        launch_arguments={}.items()
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([autoware_launch_prefix,
                                       '/launch/cargo_demo_launch/cargo_localization.launch.py']),
        launch_arguments={}.items()
    )

    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([autoware_launch_prefix,
                                       '/launch/cargo_demo_launch/cargo_planning.launch.py']),
        launch_arguments={}.items()
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([autoware_launch_prefix,
                                       '/launch/cargo_demo_launch/cargo_perception.launch.py']),
        launch_arguments={}.items()
    )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([autoware_launch_prefix,
                                       '/launch/cargo_demo_launch/cargo_visualization.launch.py']),
        launch_arguments={}.items()
    )

    return [
        mapping,
        vehicle,
        sensors,
        localization,
        planning,
        perception,
        visualization
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
