# Copyright 2020-2021, The Autoware Foundation
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

    euclidean_cluster_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/euclidean_cluster.param.yaml')
    ray_ground_classifier_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/ray_ground_classifier.param.yaml')
    scan_downsampler_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/scan_downsampler.param.yaml')

    lanelet2_map_provider_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/lanelet2_map_provider.param.yaml')

    object_collision_estimator_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/object_collision_estimator.param.yaml')

    off_map_obstacles_filter_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/off_map_obstacles_filter.param.yaml')

    vehicle_characteristics_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/vehicle_characteristics.param.yaml')

    # behavior_path_planner params

    avoidance_param_path = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config/avoidance',
        'avoidance.param.yaml')
    lane_change_param_path = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config/lane_change',
        'lane_change.param.yaml')
    lane_following_param_path = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config/lane_following',
        'lane_following.param.yaml')
    pull_out_param_path = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config/pull_out',
        'pull_out.param.yaml')
    pull_over_param_path = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config/pull_over',
        'pull_over.param.yaml')
    side_shift_param_path = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config/side_shift',
        'side_shift.param.yaml')
    behavior_path_planner_param_path = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config',
        'behavior_path_planner.param.yaml')
    vehicle_constants_param_path = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config',
        'vehicle_constants.param.yaml')
    bt_tree_config_path_value = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config',
        'behavior_path_planner_tree.xml')

    # behavior_velocity_planner params

    blind_spot_param_path = os.path.join(
        get_package_share_directory('behavior_velocity_planner_nodes'),
        'config',
        'blind_spot.param.yaml')
    crosswalk_param_path = os.path.join(
        get_package_share_directory('behavior_velocity_planner_nodes'),
        'config',
        'crosswalk.param.yaml')
    detection_area_param_path = os.path.join(
        get_package_share_directory('behavior_velocity_planner_nodes'),
        'config',
        'detection_area.param.yaml')
    intersection_param_path = os.path.join(
        get_package_share_directory('behavior_velocity_planner_nodes'),
        'config',
        'intersection.param.yaml')
    stop_line_param_path = os.path.join(
        get_package_share_directory('behavior_velocity_planner_nodes'),
        'config',
        'stop_line.param.yaml')

    point_cloud_fusion_node_pkg_prefix = get_package_share_directory(
        'point_cloud_fusion_nodes')

    # Arguments

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

    # behavior_path_planner arguments

    avoidance_param = DeclareLaunchArgument(
        'avoidance_param_path',
        default_value=avoidance_param_path,
        description='Path to config file for behavior path planner/avoidance module'
    )
    lane_change_param = DeclareLaunchArgument(
        'lane_change_param_path',
        default_value=lane_change_param_path,
        description='Path to config file for behavior path planner/lane change module'
    )
    lane_following_param = DeclareLaunchArgument(
        'lane_following_param_path',
        default_value=lane_following_param_path,
        description='Path to config file for behavior path planner/lane following module'
    )
    pull_out_param = DeclareLaunchArgument(
        'pull_out_param_path',
        default_value=pull_out_param_path,
        description='Path to config file for behavior path planner/pull out module'
    )
    pull_over_param = DeclareLaunchArgument(
        'pull_over_param_path',
        default_value=pull_over_param_path,
        description='Path to config file for behavior path planner/pull over module'
    )
    side_shift_param = DeclareLaunchArgument(
        'side_shift_param_path',
        default_value=side_shift_param_path,
        description='Path to config file for behavior path planner/side shift module'
    )
    behavior_path_planner_param = DeclareLaunchArgument(
        'behavior_path_planner_param_path',
        default_value=behavior_path_planner_param_path,
        description='Path to config file for behavior path planner'
    )
    vehicle_constants_param = DeclareLaunchArgument(
        'vehicle_constants_param_path',
        default_value=vehicle_constants_param_path,
        description='Path to config file for vehicle constants manager'
    )

    # behavior_velocity_planner arguments

    blind_spot_param = DeclareLaunchArgument(
        'blind_spot_param_path',
        default_value=blind_spot_param_path,
        description='Path to config file for behavior velocity planner/blind spot module'
    )
    crosswalk_param = DeclareLaunchArgument(
        'crosswalk_param_path',
        default_value=crosswalk_param_path,
        description='Path to config file for behavior velocity planner/crosswalk module'
    )
    detection_area_param = DeclareLaunchArgument(
        'detection_area_param_path',
        default_value=detection_area_param_path,
        description='Path to config file for behavior velocity planner/detection area module'
    )
    intersection_param = DeclareLaunchArgument(
        'intersection_param_path',
        default_value=intersection_param_path,
        description='Path to config file for behavior velocity planner/intersection module'
    )
    stop_line_param = DeclareLaunchArgument(
        'stop_line_param_path',
        default_value=stop_line_param_path,
        description='Path to config file for behavior velocity planner/stop line module'
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
    behavior_path_planner = Node(
        package='behavior_path_planner',
        name='behavior_path_planner_node',
        namespace='planning',
        executable='behavior_path_planner_exe',
        output='screen',
        parameters=[
            LaunchConfiguration('avoidance_param_path'),
            LaunchConfiguration('lane_change_param_path'),
            LaunchConfiguration('lane_following_param_path'),
            LaunchConfiguration('pull_out_param_path'),
            LaunchConfiguration('pull_over_param_path'),
            LaunchConfiguration('side_shift_param_path'),
            LaunchConfiguration('behavior_path_planner_param_path'),
            LaunchConfiguration('vehicle_constants_param_path'),
            {
                'bt_tree_config_path': bt_tree_config_path_value
            }
        ],
        remappings=[
            ('HAD_Map_Client', '/had_maps/HAD_Map_Service'),
            ('~/input/route', '/planning/global_path'),
            ('~/input/perception', '/perception/object_recognition/objects'),
            ('~/input/velocity', '/vehicle/vehicle_kinematic_state'),
            ('~/output/path', 'path_with_lane_id')
        ]
    )
    behavior_velocity_planner_nodes = Node(
        package='behavior_velocity_planner_nodes',
        name='behavior_velocity_planner_nodes',
        namespace='planning',
        executable='behavior_velocity_planner_nodes_exe',
        output='screen',
        parameters=[
            LaunchConfiguration('blind_spot_param_path'),
            LaunchConfiguration('crosswalk_param_path'),
            LaunchConfiguration('detection_area_param_path'),
            LaunchConfiguration('intersection_param_path'),
            LaunchConfiguration('stop_line_param_path'),
            LaunchConfiguration('vehicle_constants_param_path'),
            {
                'launch_stop_line': True,
                'launch_crosswalk': True,
                'launch_traffic_light': True,
                'launch_intersection': True,
                'launch_blind_spot': True,
                'launch_detection_area': True,
                'forward_path_length': 200.0,
                'backward_path_length': 0.0,
                'max_accel': -2.8,
                'delay_response_time': 1.3,
                'use_sim_time': True
            }
        ],
        remappings=[
            ('HAD_Map_Client', '/had_maps/HAD_Map_Service'),
            ('~/input/path_with_lane_id', '/planning/path_with_lane_id'),
            ('~/input/dynamic_objects', '/perception/object_recognition/objects'),
            ('~/input/vehicle_velocity', '/vehicle/vehicle_kinematic_state'),
            ('~/output/trajectory', 'trajectory'),
            ('/clock', '/lgsvl/clock')
        ]
    )

    return LaunchDescription([
        euclidean_cluster_param,
        ray_ground_classifier_param,
        scan_downsampler_param,
        with_obstacles_param,
        lanelet2_map_provider_param,
        object_collision_estimator_param,
        off_map_obstacles_filter_param,
        vehicle_characteristics_param,
        avoidance_param,
        lane_change_param,
        lane_following_param,
        pull_out_param,
        pull_over_param,
        side_shift_param,
        behavior_path_planner_param,
        blind_spot_param,
        crosswalk_param,
        detection_area_param,
        intersection_param,
        stop_line_param,
        vehicle_constants_param,
        euclidean_clustering,
        ray_ground_classifier,
        scan_downsampler,
        point_cloud_fusion_node,
        lanelet2_map_provider,
        lanelet2_map_visualizer,
        global_planner,
        object_collision_estimator,
        off_map_obstacles_filter,
        behavior_path_planner,
        behavior_velocity_planner_nodes
    ])
