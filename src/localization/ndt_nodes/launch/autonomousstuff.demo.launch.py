import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
import ament_index_python
import os
import launch.launch_description_sources

def get_param(package_name, shared_file):
    return os.path.join(ament_index_python.get_package_share_directory(package_name), shared_file)

lgsvl_launch_file_path = get_param('lgsvl_interface', 'lgsvl.launch.py')
lgsvl_interface = launch.actions.IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource(lgsvl_launch_file_path)
)

rviz2 = launch_ros.actions.Node(
    package='rviz2',
    node_executable='rviz2',
    node_name='rviz2',
    arguments=['-d', get_param("ndt_nodes", "demo_ndt.rviz")])

ndt_node = launch_ros.actions.Node(
    package='ndt_nodes',
    node_executable='p2d_ndt_localizer_exe',
    node_name='p2d_ndt_localizer_node',
    parameters=[get_param("ndt_nodes", "p2d_ndt_node.default.param.yaml")])

simulator = launch.actions.ExecuteProcess(cmd=["/opt/lgsvl/simulator"], shell=True)
rosbridge = launch.actions.ExecuteProcess(cmd=["rosbridge"], shell=True)

map_publisher = launch_ros.actions.Node(
    package='ndt_nodes',
    node_executable='ndt_map_publisher_exe',
    node_name='ndt_map_publisher_node',
    parameters=[get_param("ndt_nodes", "map_publisher.example.yaml"),
                {"map_file_name" : get_param("ndt_nodes", "AutonomouStuff.pcd")}])

voxel_grid = launch_ros.actions.Node(
    package='voxel_grid_nodes',
    node_executable='voxel_grid_cloud_node_exe',
    node_name='voxel_grid_cloud_node',
    parameters=[get_param("ndt_nodes", "vlp16_lexus_centroid.param.yaml")])  # TODO(yunus.caliskan): fix voxel grid param installation

def generate_launch_description():
    return launch.LaunchDescription([rviz2, simulator, rosbridge, lgsvl_interface, voxel_grid, ndt_node, map_publisher])
