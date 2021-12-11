from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

import os

def IfEqualsCondition(arg_name: str, value: str):
    return IfCondition(PythonExpression([
        '"', LaunchConfiguration(arg_name), '" == "', value, '"'
    ]))

def generate_launch_description():

    # package paths
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')
    slam_toolbox_pkg_prefix = get_package_share_directory('slam_toolbox')

    # parameters
    mapping_param_file = os.path.join(
        f1tenth_launch_pkg_prefix, "param/mapper_params_online_async.param.yaml"
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

    with_rviz_param = DeclareLaunchArgument(
        'with_rviz',
        default_value='True',
        description='Launch rviz in addition to other nodes'
    )

    vehicle_interface_mode = DeclareLaunchArgument(
        'vehicle_interface',
        default_value='svl',
        description='Launch rviz in addition to other nodes'
    )

    rviz_cfg_path = os.path.join(f1tenth_launch_pkg_prefix,
                                 'rviz2', 'f1tenth.rviz')
    rviz_cfg_path_param = DeclareLaunchArgument(
        'rviz_cfg_path_param',
        default_value=rviz_cfg_path,
        description='Launch RVIZ2 with the specified config file'
    )

    # nodes
    vehicle_launch_svl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_launch_pkg_prefix,
                         'launch/f1tenth_vehicle_svl.launch.py'),
        ),
        launch_arguments={
            'with_joy': LaunchConfiguration('with_joy'),
        }.items(),
        condition=IfEqualsCondition("vehicle_interface", "svl")
    )

    vehicle_launch_vesc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_launch_pkg_prefix,
                         'launch/f1tenth_vehicle_vesc.launch.py'),
        ),
        condition=IfEqualsCondition("vehicle_interface", "vesc")
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
        condition=IfCondition(LaunchConfiguration('with_rviz')),
        arguments=['-d', LaunchConfiguration("rviz_cfg_path_param")]
    )

    return LaunchDescription([
        with_joy_param,
        with_rviz_param,
        vehicle_interface_mode,
        mapping_param,
        rviz_cfg_path_param,
        vehicle_launch_svl,
        vehicle_launch_vesc,
        slam_launch,
        rviz2
    ])