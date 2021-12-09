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
    slam_toolbox_pkg_prefix = get_package_share_directory('slam_toolbox')

    # parameters
    mapping_param_file = os.path.join(
        autoware_demos_pkg_prefix, "param/f1tenth/mapper_params_online_async.param.yaml"
    )
    mapping_param = DeclareLaunchArgument(
        "mapping_param_file",
        default_value=mapping_param_file,
        description="Path to config file for mapping nodes",
    )

    rviz_cfg_path = os.path.join(autoware_demos_pkg_prefix,
                                 'rviz2', 'f1tenth.rviz')
    rviz_cfg_path_param = DeclareLaunchArgument(
        'rviz_cfg_path_param',
        default_value=rviz_cfg_path,
        description='Launch RVIZ2 with the specified config file'
    )

    # nodes
    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(autoware_demos_pkg_prefix,
                         'launch/f1tenth_demo/f1tenth_vehicle.launch.py'),
        )
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg_prefix,
                         'launch/online_async_launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('mapping_param_file'),
        }.items()
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration("rviz_cfg_path_param")]
    )

    return LaunchDescription([
        mapping_param,
        rviz_cfg_path_param,
        vehicle_launch,
        slam_launch,
        rviz2
    ])