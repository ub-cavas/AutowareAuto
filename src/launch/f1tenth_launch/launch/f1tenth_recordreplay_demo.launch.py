from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import os

def generate_launch_description():

    # package paths
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')

    # params
    rviz_cfg_path = os.path.join(f1tenth_launch_pkg_prefix,
                                 'rviz2', 'f1tenth.rviz')
    rviz_cfg_path_param = DeclareLaunchArgument(
        'rviz_cfg_path_param',
        default_value=rviz_cfg_path,
        description='Launch RVIZ2 with the specified config file'
    )

    map_file_path = os.path.join(f1tenth_launch_pkg_prefix, 'data/red_bull_ring_racetrack.yaml')
    map_file = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Path to 2D map config file'
    )

    with_joy_param = DeclareLaunchArgument(
        'with_joy',
        default_value='False',
        description='Launch joystick_interface in addition to other nodes'
    )

    # Nodes
    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_launch_pkg_prefix,
                         'launch/f1tenth_vehicle.launch.py'),
        ),
        launch_arguments={
            'with_joy': LaunchConfiguration('with_joy'),
        }.items()
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_launch_pkg_prefix,
                         'launch/f1tenth_localization.launch.py'),
        ),
        launch_arguments={
            'map': LaunchConfiguration('map')
        }.items()
    )

    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_launch_pkg_prefix,
                         'launch/f1tenth_planning.launch.py'),
        )
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration("rviz_cfg_path_param")]
    )

    return LaunchDescription([
        with_joy_param,
        map_file,
        vehicle_launch,
        localization_launch,
        planning_launch,
        rviz_cfg_path_param,
        rviz2
    ])