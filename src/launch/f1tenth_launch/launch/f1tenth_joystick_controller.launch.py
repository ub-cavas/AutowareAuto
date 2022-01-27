from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import os

def generate_launch_description():
    f1tenth_launch_pkg = get_package_share_directory('f1tenth_launch')

    control_command_param = DeclareLaunchArgument(
        'control_command',
        default_value="basic",  # use "raw", "basic" or "high_level"
        description='command control mode topic name')

    joy_translator_param_file = os.path.join(f1tenth_launch_pkg, 'param/logitech_f310_basic.param.yaml')
    joy_translator_param = DeclareLaunchArgument(
        'joy_translator_param',
        default_value=joy_translator_param_file,
        description='Path to config file for joystick translator')


    # joystick driver node
    joy = Node(
        package='joy_linux',
        executable='joy_linux_node',
        output='screen')

    # joystick translator node
    joy_translator = Node(
        package='joystick_vehicle_interface_nodes',
        executable='joystick_vehicle_interface_node_exe',
        output='screen',
        parameters=[
            LaunchConfiguration('joy_translator_param'),
            {"control_command": LaunchConfiguration('control_command')}
        ],
        remappings=[
            ("basic_command", "/vehicle/vehicle_command"),
            ("raw_command", "/vehicle/raw_command"),
            ("state_command", "/vehicle/state_command")
        ])


    return LaunchDescription([
        control_command_param,
        joy,
        joy_translator_param,
        joy_translator,
    ])