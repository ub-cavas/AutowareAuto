from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import os

def generate_launch_description():
    demos_pkg_prefix = get_package_share_directory('autoware_demos')
    joystick_vehicle_interface_prefix = get_package_share_directory('joystick_vehicle_interface_nodes')

    # params
    lgsvl_param_file = os.path.join(demos_pkg_prefix, 'param/f1tenth/lgsvl_interface.param.yaml')
    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param_file',
        default_value=lgsvl_param_file,
        description='Path to config file for LGSVL Interface'
    )

    joy_translator_param_file = os.path.join(demos_pkg_prefix, 'param/f1tenth/logitech_f310_basic.param.yaml')
    joy_translator_param = DeclareLaunchArgument(
        'joy_translator_param',
        default_value=joy_translator_param_file,
        description='Path to config file for LGSVL Interface'
    )

    with_joy_param = DeclareLaunchArgument(
        'with_joy',
        default_value='False',
        description='Launch joystick_interface in addition to other nodes'
    )

    # Nodes
    urdf_path = os.path.join(demos_pkg_prefix, 'urdf/f1tenth.urdf')
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()
    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
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

    joystick_vehicle_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(demos_pkg_prefix,
                         'launch/f1tenth_demo/f1tenth_joystick_controller.launch.py')
        ),
        launch_arguments={
            'control_command':'basic',
            'joy_translator_param': LaunchConfiguration('joy_translator_param'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('with_joy'))
    )

    lgsvl_bridge = ExecuteProcess(cmd=["lgsvl_bridge"])

    return LaunchDescription([
        with_joy_param,
        joy_translator_param,
        joystick_vehicle_interface,
        lgsvl_interface_param,
        lgsvl_bridge,
        urdf_publisher,
        lgsvl_interface
    ])