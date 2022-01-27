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
    f1tenth_launch_pkg = get_package_share_directory('f1tenth_launch')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    # parameters
    mapping_param_file = os.path.join(
        f1tenth_launch_pkg, "param/mapper_params_online_async.param.yaml"
    )
    mapping_param = DeclareLaunchArgument(
        "mapping_param_file",
        default_value=mapping_param_file,
        description="Path to config file for mapping nodes",
    )

    with_joy_param = DeclareLaunchArgument(
        'with_joy',
        default_value='False',
        description='Launch joystick_interface in addition to other nodes'
    )

    rviz_cfg_path = os.path.join(f1tenth_launch_pkg,
                                 'rviz2', 'f1tenth.rviz')
    rviz_cfg_path_param = DeclareLaunchArgument(
        'rviz_cfg_path_param',
        default_value=rviz_cfg_path,
        description='Launch RVIZ2 with the specified config file'
    )

    # nodes
    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_launch_pkg,
                         'launch/f1tenth_vehicle.launch.py'),
        ),
        launch_arguments={
            'with_joy': LaunchConfiguration('with_joy'),
        }.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg,
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
        with_joy_param,
        mapping_param,
        rviz_cfg_path_param,
        vehicle_launch,
        slam_launch,
        rviz2
    ])