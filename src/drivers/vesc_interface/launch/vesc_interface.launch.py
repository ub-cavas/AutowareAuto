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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

"""
Example launch file for a new package.

Note: Does not work in ROS2 dashing!
"""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory
from os import path as osp


def get_share_file(package_name, file_name):
    """
    Find the path to a given package name, and append with a file_name.

    :param package_name: The name of the package
    :param file_name: relative location of the file wrt to package
    :returns string: The appended file name
    """
    return osp.join(
        get_package_share_directory(package_name=package_name),
        file_name
    )


def generate_launch_description():
    """
    Generate Launch Description.

    Launch VESC_driver node and VESC_interface node for interfacing
    Autoware.Auto with F1TENTH car.
    """
    # Launch VESC_driver node
    vesc_driver = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        namespace='vehicle',
        output='screen',
        parameters=[get_share_file('vesc_driver', 'params/vesc_config.yaml')]
    )

    # Launch VESC_interface'
    # Default VESC_interface params
    vesc_interface_params = DeclareLaunchArgument(
        'vesc_interface_param',
        default_value=[
            get_share_file('vesc_interface', 'param/vesc_config.param.yaml')
        ],
        description="Path to config file for vesc interface"
    )

    vesc_interface = Node(
        package='vesc_interface',
        executable='vesc_interface_node_exe',
        namespace='vehicle',
        output='screen',
        parameters=[
            LaunchConfiguration('vesc_interface_param')
            # paramFile
        ]
    )

    ld = LaunchDescription([
        vesc_driver,
        vesc_interface_params,
        vesc_interface
    ])
    return ld
