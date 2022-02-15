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

import launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    """Generate launch description with a single component."""
    dataspeed_ford_interface_params_file = LaunchConfiguration(
        "dataspeed_ford_interface_params",
        default=[
            get_package_share_directory('dataspeed_ford_interface'),
            '/param/defaults.param.yaml'
        ])

    return launch.LaunchDescription([
        Node(
            package='dataspeed_ford_interface',
            executable='dataspeed_ford_interface_node_exe',
            output='screen',
            namespace='vehicle',
            parameters=[dataspeed_ford_interface_params_file],
        )
    ])

generate_launch_description()