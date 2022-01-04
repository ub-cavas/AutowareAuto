# Copyright 2020 the Autoware Foundation
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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

"""Launch LaneLet2 Global Path Planner nodes."""

import os

from ament_index_python.packages import get_package_share_directory
import launch.substitutions
from launch_ros.actions import Node
import yaml

def generate_launch_description():

    # params
    avoidance_param_path = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config/avoidance',
        'avoidance.param.yaml',
    )
    with open(avoidance_param_path, 'r') as f:
        avoidance_param = yaml.safe_load(f)['/**']['ros__parameters']

    lane_change_param_path = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config/lane_change',
        'lane_change.param.yaml',
    )
    with open(lane_change_param_path, 'r') as f:
        lane_change_param = yaml.safe_load(f)['/**']['ros__parameters']

    lane_following_param_path = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config/lane_following',
        'lane_following.param.yaml',
    )
    with open(lane_following_param_path, 'r') as f:
        lane_following_param = yaml.safe_load(f)['/**']['ros__parameters']

    pull_out_param_path = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config/pull_out',
        'pull_out.param.yaml',
    )
    with open(pull_out_param_path, 'r') as f:
        pull_out_param = yaml.safe_load(f)['/**']['ros__parameters']

    pull_over_param_path = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config/pull_over',
        'pull_over.param.yaml',
    )
    with open(pull_over_param_path, 'r') as f:
        pull_over_param = yaml.safe_load(f)['/**']['ros__parameters']

    side_shift_param_path = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config/side_shift',
        'side_shift.param.yaml',
    )
    with open(side_shift_param_path, 'r') as f:
        side_shift_param = yaml.safe_load(f)['/**']['ros__parameters']

    behavior_path_planner_param_path = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config',
        'behavior_path_planner.param.yaml',
    )
    with open(behavior_path_planner_param_path, 'r') as f:
        behavior_path_planner_param = yaml.safe_load(f)['/**']['ros__parameters']

    vehicle_constants_param_path = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config',
        'vehicle_constants.param.yaml',
    )
    with open(vehicle_constants_param_path, 'r') as f:
        vehicle_constants_param = yaml.safe_load(f)['/**']['ros__parameters']

    bt_tree_config_path_value = os.path.join(
        get_package_share_directory('behavior_path_planner'),
        'config',
        'behavior_path_planner_tree.xml',
    )

    # execution definition.
    behavior_path_planner = Node(
        package='behavior_path_planner',
        name='behavior_path_planner_node',
        namespace='planning',
        executable='behavior_path_planner_exe',
        output='screen',
        remappings=[('HAD_Map_Client', '/had_maps/HAD_Map_Service'),
                    ('~/input/route', '/planning/global_path'),
                    ('~/input/perception', '/perception/object_recognition/objects'),
                    ('~/input/velocity', '/vehicle/vehicle_kinematic_state'),
                    ('~/output/path', 'path_with_lane_id')],
        parameters=[
            {
                'bt_tree_config_path': bt_tree_config_path_value
            },
            avoidance_param,
            lane_change_param,
            lane_following_param,
            pull_out_param,
            pull_over_param,
            side_shift_param,
            behavior_path_planner_param,
            vehicle_constants_param
            ],
    )
    return launch.LaunchDescription([behavior_path_planner])
