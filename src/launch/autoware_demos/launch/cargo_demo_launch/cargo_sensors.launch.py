# Copyright 2021 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """
    Launch sensos nodes.

     * point_cloud_filter_transform
     * point_cloud_fusion_node
     * voxel_grid_node
    """
    autoware_auto_launch_pkg_prefix = get_package_share_directory(
        'autoware_auto_launch')
    point_cloud_filter_transform_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/point_cloud_filter_transform.param.yaml')
    voxel_grid_node_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/voxel_grid_node.param.yaml')

    # Arguments
    point_cloud_filter_transform_param = DeclareLaunchArgument(
        'point_cloud_filter_transform_param_file',
        default_value=point_cloud_filter_transform_param_file,
        description='Path to config file for Point Cloud Filter/Transform Nodes'
    )
    voxel_grid_node_param = DeclareLaunchArgument(
        'voxel_grid_node_param_file',
        default_value=voxel_grid_node_param_file,
        description='Path to config file for lidar scan downsampler'
    )

    # Nodes
    filter_transform_vlp16_front = Node(
        package='point_cloud_filter_transform_nodes',
        executable='point_cloud_filter_transform_node_exe',
        name='filter_transform_vlp16',
        namespace='lidars',
        parameters=[LaunchConfiguration('point_cloud_filter_transform_param_file')],
        remappings=[("points_in", "points_xyzi")]
    )

    voxel_grid_node = Node(
        package='voxel_grid_nodes',
        executable='voxel_grid_node_exe',
        namespace='lidars',
        name='voxel_grid_cloud_node',
        parameters=[LaunchConfiguration('voxel_grid_node_param_file')],
        remappings=[
            ("points_in", "points_filtered"),
            ("points_downsampled", "points_downsampled")
        ]
    )

    return LaunchDescription([
        point_cloud_filter_transform_param,
        voxel_grid_node_param,
        filter_transform_vlp16_front,
        voxel_grid_node,
    ])
