# Copyright 2022 Arm Ltd
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    api_endpoint_address_param = DeclareLaunchArgument(
        'api_endpoint_address',
        default_value='localhost',
        description='Address for the LG SVL simulation API endpoint',
    )
    api_endpoint_port_param = DeclareLaunchArgument(
        'api_endpoint_port',
        default_value='8181',
        description='Port for the LG SVL simulation API endpoint',
    )
    ros2_bridge_address_param = DeclareLaunchArgument(
        'ros2_bridge_address',
        default_value='localhost',
        description='Address for the ROS2 bridge',
    )
    ros2_bridge_port_param = DeclareLaunchArgument(
        'ros2_bridge_port',
        default_value='9090',
        description='Port for the ROS2 bridge',
    )
    simulation_param = DeclareLaunchArgument(
        'simulation_params',
        description='Simulation parameters',
    )
    override_simulation_param = DeclareLaunchArgument(
        'override_simulation_params',
        default_value='',
        description='Override simulation parameters',
    )

    # Simulation node
    simulation_node = Node(
        package='lgsvl_simulation',
        executable='lgsvl_simulation_main.py',
        name='lgsvl_simulation_node',
        namespace='drivers',
        parameters=[
            LaunchConfiguration('simulation_params'),
            LaunchConfiguration('override_simulation_params'),
            {
                'api_endpoint': {
                    'address': LaunchConfiguration('api_endpoint_address'),
                    'port': LaunchConfiguration('api_endpoint_port'),
                },
                'vehicle.bridge': {
                    'address': LaunchConfiguration('ros2_bridge_address'),
                    'port': LaunchConfiguration('ros2_bridge_port'),
                }
            },
        ]
    )

    return LaunchDescription([
        api_endpoint_address_param,
        api_endpoint_port_param,
        ros2_bridge_address_param,
        ros2_bridge_port_param,
        simulation_param,
        override_simulation_param,
        simulation_node
    ])
