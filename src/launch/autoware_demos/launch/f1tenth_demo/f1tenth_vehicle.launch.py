from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import os

def generate_launch_description():
    demos_pkg_prefix = get_package_share_directory('autoware_demos')
    autoware_launch_pkg_prefix = get_package_share_directory('autoware_auto_launch')
    lgsvl_param_file = os.path.join(demos_pkg_prefix, 'param/f1tenth/lgsvl_interface.param.yaml')

    urdf_path = os.path.join(demos_pkg_prefix, 'urdf/f1tenth.urdf')
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()
    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
    )

    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param_file',
        default_value=lgsvl_param_file,
        description='Path to config file for LGSVL Interface'
    )

    odom_tf_node = Node(
            package='svl_robot_bringup',
            executable='odom_tf',
        )

    lgsvl_interface = Node(
        package='lgsvl_interface',
        executable='lgsvl_interface_exe',
        namespace='vehicle',
        output='screen',
        parameters=[
          LaunchConfiguration('lgsvl_interface_param_file'),
          {"lgsvl.publish_tf": True}
        ],
        remappings=[
            ("vehicle_control_cmd", "/lgsvl/vehicle_control_cmd"),
            ("vehicle_state_cmd", "/lgsvl/vehicle_state_cmd"),
            ("state_report", "/lgsvl/state_report"),
            ("state_report_out", "/vehicle/state_report"),
            ("gnss_odom", "/lgsvl/gnss_odom"),
            ("vehicle_odom", "/lgsvl/vehicle_odom")
        ]
    )

    lgsvl_bridge = ExecuteProcess(cmd=["lgsvl_bridge"])

    return LaunchDescription([
        lgsvl_interface_param,
        lgsvl_bridge,
        urdf_publisher,
        lgsvl_interface
    ])