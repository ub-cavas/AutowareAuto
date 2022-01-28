# Copyright 2022, The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Modules for Milestone 3 of the AVP 2020 Demo."""

from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """
    Launch all nodes defined in the architecture for Milestone 3 of the AVP 2020 Demo.

    More details about what is included can
    be found at https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/milestones/25.
    """
    avp_demo_pkg_prefix = get_package_share_directory('autoware_demos')
    autoware_launch_pkg_prefix = get_package_share_directory('autoware_auto_launch')
    global_vel_planner_prefix = get_package_share_directory('global_velocity_planner')

    euclidean_cluster_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/euclidean_cluster.param.yaml')
    ray_ground_classifier_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/ray_ground_classifier.param.yaml')
    scan_downsampler_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/scan_downsampler.param.yaml')

    lanelet2_map_provider_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/lanelet2_map_provider.param.yaml')

    global_velocity_planner_param_file = os.path.join(
        global_vel_planner_prefix, 'param/defaults.param.yaml')
    costmap_generator_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/costmap_generator.param.yaml')
    freespace_planner_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/freespace_planner.param.yaml')
    object_collision_estimator_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/object_collision_estimator.param.yaml')
    off_map_obstacles_filter_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/off_map_obstacles_filter.param.yaml')

    vehicle_characteristics_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/vehicle_characteristics.param.yaml')
    vehicle_constants_manager_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/lexus_rx_hybrid_2016.param.yaml')

    point_cloud_fusion_node_pkg_prefix = get_package_share_directory(
        'point_cloud_fusion_nodes')

    # Arguments

    global_velocity_planner_param = DeclareLaunchArgument(
        'global_velocity_planner_param_file',
        default_value=global_velocity_planner_param_file,
        description='Path to parameter file for global velocity planner'
    )

    euclidean_cluster_param = DeclareLaunchArgument(
        'euclidean_cluster_param_file',
        default_value=euclidean_cluster_param_file,
        description='Path to config file for Euclidean Clustering'
    )
    ray_ground_classifier_param = DeclareLaunchArgument(
        'ray_ground_classifier_param_file',
        default_value=ray_ground_classifier_param_file,
        description='Path to config file for Ray Ground Classifier'
    )
    with_obstacles_param = DeclareLaunchArgument(
        'with_obstacles',
        default_value='True',
        description='Enable obstacle detection'
    )
    scan_downsampler_param = DeclareLaunchArgument(
        'scan_downsampler_param_file',
        default_value=scan_downsampler_param_file,
        description='Path to config file for lidar scan downsampler'
    )
    lanelet2_map_provider_param = DeclareLaunchArgument(
        'lanelet2_map_provider_param_file',
        default_value=lanelet2_map_provider_param_file,
        description='Path to parameter file for Lanelet2 Map Provider'
    )
    costmap_generator_param = DeclareLaunchArgument(
        'costmap_generator_param_file',
        default_value=costmap_generator_param_file,
        description='Path to parameter file for costmap generator'
    )
    freespace_planner_param = DeclareLaunchArgument(
        'freespace_planner_param_file',
        default_value=freespace_planner_param_file,
        description='Path to parameter file for freespace_planner'
    )
    object_collision_estimator_param = DeclareLaunchArgument(
        'object_collision_estimator_param_file',
        default_value=object_collision_estimator_param_file,
        description='Path to parameter file for object collision estimator'
    )
    off_map_obstacles_filter_param = DeclareLaunchArgument(
        'off_map_obstacles_filter_param_file',
        default_value=off_map_obstacles_filter_param_file,
        description='Path to parameter file for off-map obstacle filter'
    )
    vehicle_characteristics_param = DeclareLaunchArgument(
        'vehicle_characteristics_param_file',
        default_value=vehicle_characteristics_param_file,
        description='Path to config file for vehicle characteristics'
    )
    vehicle_constants_manager_param = DeclareLaunchArgument(
        'vehicle_constants_manager_param_file',
        default_value=vehicle_constants_manager_param_file,
        description='Path to parameter file for vehicle_constants_manager'
    )

    # Nodes

    euclidean_clustering = Node(
        package='euclidean_cluster_nodes',
        executable='euclidean_cluster_node_exe',
        namespace='perception',
        condition=IfCondition(LaunchConfiguration('with_obstacles')),
        parameters=[LaunchConfiguration('euclidean_cluster_param_file')],
        remappings=[
            ("points_in", "points_nonground")
        ]
    )
    # point cloud fusion runner to fuse front and rear lidar

    point_cloud_fusion_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(point_cloud_fusion_node_pkg_prefix,
                         'launch/vlp16_sim_lexus_pc_fusion.launch.py'))
    )
    ray_ground_classifier = Node(
        package='ray_ground_classifier_nodes',
        executable='ray_ground_classifier_cloud_node_exe',
        namespace='perception',
        condition=IfCondition(LaunchConfiguration('with_obstacles')),
        parameters=[LaunchConfiguration('ray_ground_classifier_param_file')],
        remappings=[("points_in", "/lidars/points_fused")]
    )
    scan_downsampler = Node(
        package='voxel_grid_nodes',
        executable='voxel_grid_node_exe',
        namespace='lidars',
        name='voxel_grid_cloud_node',
        parameters=[LaunchConfiguration('scan_downsampler_param_file')],
        remappings=[
            ("points_in", "points_fused"),
            ("points_downsampled", "points_fused_downsampled")
        ]
    )
    lanelet2_map_provider = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_provider_exe',
        namespace='had_maps',
        name='lanelet2_map_provider_node',
        parameters=[LaunchConfiguration('lanelet2_map_provider_param_file')]
    )
    lanelet2_map_visualizer = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_visualizer_exe',
        name='lanelet2_map_visualizer_node',
        namespace='had_maps'
    )
    global_planner = Node(
        package='lanelet2_global_planner_nodes',
        name='lanelet2_global_planner_node',
        namespace='planning',
        executable='lanelet2_global_planner_node_exe',
        remappings=[('HAD_Map_Client', '/had_maps/HAD_Map_Service'),
                    ('vehicle_kinematic_state', '/vehicle/vehicle_kinematic_state')]
    )
    costmap_generator = Node(
        package='costmap_generator_nodes',
        executable='costmap_generator_node_exe',
        name='costmap_generator_node',
        namespace='planning',
        output='screen',
        parameters=[
            LaunchConfiguration('costmap_generator_param_file'),
        ],
        remappings=[
            ('~/client/HAD_Map_Service', '/had_maps/HAD_Map_Service')
        ]
    )
    freespace_planner = Node(
        package='freespace_planner',
        executable='freespace_planner_node_exe',
        name='freespace_planner',
        namespace='planning',
        output='screen',
        parameters=[
            LaunchConfiguration('freespace_planner_param_file'),
            LaunchConfiguration('vehicle_constants_manager_param_file')
        ]
    )
    object_collision_estimator = Node(
        package='object_collision_estimator_nodes',
        name='object_collision_estimator_node',
        namespace='planning',
        executable='object_collision_estimator_node_exe',
        condition=IfCondition(LaunchConfiguration('with_obstacles')),
        parameters=[
            LaunchConfiguration('object_collision_estimator_param_file'),
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        remappings=[
            ('obstacle_topic', '/perception/lidar_bounding_boxes_filtered'),
        ]
    )
    off_map_obstacles_filter = Node(
        package='off_map_obstacles_filter_nodes',
        name='off_map_obstacles_filter_node',
        namespace='perception',
        executable='off_map_obstacles_filter_nodes_exe',
        parameters=[LaunchConfiguration('off_map_obstacles_filter_param_file')],
        output='screen',
        remappings=[
            ('bounding_boxes_in', 'lidar_bounding_boxes'),
            ('bounding_boxes_out', 'lidar_bounding_boxes_filtered'),
            ('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
        ]
    )
    global_velocity_planner = Node(
        package='global_velocity_planner',
        name='global_velocity_planner_node',
        namespace='planning',
        output='screen',
        executable='global_velocity_planner_node_exe',
        parameters=[
            LaunchConfiguration('global_velocity_planner_param_file'),
        ],
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
                    ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
                    ('route', 'global_path')]
    )

    return LaunchDescription([
        euclidean_cluster_param,
        ray_ground_classifier_param,
        scan_downsampler_param,
        with_obstacles_param,
        lanelet2_map_provider_param,
        costmap_generator_param,
        freespace_planner_param,
        object_collision_estimator_param,
        global_velocity_planner_param,
        off_map_obstacles_filter_param,
        vehicle_characteristics_param,
        vehicle_constants_manager_param,
        euclidean_clustering,
        ray_ground_classifier,
        scan_downsampler,
        point_cloud_fusion_node,
        lanelet2_map_provider,
        lanelet2_map_visualizer,
        global_planner,
        costmap_generator,
        freespace_planner,
        object_collision_estimator,
        off_map_obstacles_filter,
        global_velocity_planner,
    ])
