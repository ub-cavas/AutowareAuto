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
    autoware_demos_pkg_prefix = get_package_share_directory('autoware_demos')

    # params
    rviz_cfg_path = os.path.join(autoware_demos_pkg_prefix,
                                 'rviz2', 'f1tenth.rviz')
    rviz_cfg_path_param = DeclareLaunchArgument(
        'rviz_cfg_path_param',
        default_value=rviz_cfg_path,
        description='Launch RVIZ2 with the specified config file'
    )

    # Nodes
    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(autoware_demos_pkg_prefix,
                         'launch/f1tenth_demo/f1tenth_vehicle.launch.py'),
        )
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(autoware_demos_pkg_prefix,
                         'launch/f1tenth_demo/f1tenth_localization.launch.py'),
        )
    )

    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(autoware_demos_pkg_prefix,
                         'launch/f1tenth_demo/f1tenth_planning.launch.py'),
        )
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration("rviz_cfg_path_param")]
    )

    return LaunchDescription([
        vehicle_launch,
        localization_launch,
        planning_launch,
        rviz_cfg_path_param,
        rviz2
    ])