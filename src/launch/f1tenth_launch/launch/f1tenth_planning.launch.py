from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import os

def generate_launch_description():
    f1tenth_launch_pkg = get_package_share_directory('f1tenth_launch')

    # parameters
    pure_pursuit_param_file = os.path.join(f1tenth_launch_pkg, 'param/pure_pursuit.param.yaml')
    recordreplay_planner_param_file = os.path.join(f1tenth_launch_pkg, 'param/recordreplay_planner.param.yaml')

    pure_pursuit_param = DeclareLaunchArgument(
        'pure_pursuit_param_file',
        default_value=pure_pursuit_param_file,
        description='Path to config file for pure pursuit controller'
    )

    recordreplay_planner_param = DeclareLaunchArgument(
        'recordreplay_planner_param_file',
        default_value=recordreplay_planner_param_file,
        description='Path to config file for record/replay planner'
    )

    # Nodes
    pure_pursuit = Node(
        package='pure_pursuit_nodes',
        executable='pure_pursuit_node_exe',
        name='pure_pursuit_node',
        output="screen",
        parameters=[
            LaunchConfiguration('pure_pursuit_param_file'),
        ],
        remappings=[
            ("current_pose", "/vehicle/vehicle_kinematic_state"),
            ("trajectory", "/planning/trajectory"),
            ("ctrl_cmd", "/vehicle/vehicle_command"),
            ("ctrl_diag", "/control/control_diagnostic"),
        ]
    )
    recordreplay_planner = Node(
        package='recordreplay_planner_nodes',
        executable='recordreplay_planner_node_exe',
        name='recordreplay_planner',
        namespace='planning',
        parameters=[
            LaunchConfiguration('recordreplay_planner_param_file')
        ],
        remappings=[
            ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
            ('planned_trajectory', '/planning/trajectory')
        ]
    )

    return LaunchDescription([
        pure_pursuit_param,
        pure_pursuit,
        recordreplay_planner_param,
        recordreplay_planner,
    ])