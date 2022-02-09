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

import os

from ament_index_python.packages import get_package_share_directory
import launch
import yaml
import launch_ros.actions


def generate_launch_description():
    # behavior velocity planner
    vehicle_constants_param_path = os.path.join(
        get_package_share_directory('behavior_velocity_planner_nodes'),
        'config',
        'vehicle_constants.param.yaml',
    )
    with open(vehicle_constants_param_path, 'r') as f:
        vehicle_constants_param = yaml.safe_load(f)['/**']['ros__parameters']

    blind_spot_param_path = os.path.join(
        get_package_share_directory('behavior_velocity_planner_nodes'),
        'config',
        'blind_spot.param.yaml',
    )
    with open(blind_spot_param_path, 'r') as f:
        blind_spot_param = yaml.safe_load(f)['/**']['ros__parameters']

    crosswalk_param_path = os.path.join(
        get_package_share_directory('behavior_velocity_planner_nodes'),
        'config',
        'crosswalk.param.yaml',
    )
    with open(crosswalk_param_path, 'r') as f:
        crosswalk_param = yaml.safe_load(f)['/**']['ros__parameters']

    detection_area_param_path = os.path.join(
        get_package_share_directory('behavior_velocity_planner_nodes'),
        'config',
        'detection_area.param.yaml',
    )
    with open(detection_area_param_path, 'r') as f:
        detection_area_param = yaml.safe_load(f)['/**']['ros__parameters']

    intersection_param_path = os.path.join(
        get_package_share_directory('behavior_velocity_planner_nodes'),
        'config',
        'intersection.param.yaml',
    )
    with open(intersection_param_path, 'r') as f:
        intersection_param = yaml.safe_load(f)['/**']['ros__parameters']

    stop_line_param_path = os.path.join(
        get_package_share_directory('behavior_velocity_planner_nodes'),
        'config',
        'stop_line.param.yaml',
    )
    with open(stop_line_param_path, 'r') as f:
        stop_line_param = yaml.safe_load(f)['/**']['ros__parameters']

    # traffic_light_param_path = os.path.join(
    #     get_package_share_directory('behavior_velocity_planner_nodes'),
    #     'config',
    #     'traffic_light.param.yaml',
    # )
    # with open(traffic_light_param_path, 'r') as f:
    #     traffic_light_param = yaml.safe_load(f)['/**']['ros__parameters']

    # virtual_traffic_light_param_path = os.path.join(
    #     get_package_share_directory('behavior_velocity_planner_nodes'),
    #     'config',
    #     'virtual_traffic_light.param.yaml',
    # )
    # with open(virtual_traffic_light_param_path, 'r') as f:
    #     virtual_traffic_light_param = yaml.safe_load(f)['/**']['ros__parameters']

    behavior_velocity_planner_nodes = launch_ros.actions.Node(
        package='behavior_velocity_planner_nodes',
        name='behavior_velocity_planner_nodes',
        namespace='planning',
        executable='behavior_velocity_planner_nodes_exe',
        output='screen',
        remappings=[('HAD_Map_Client', '/had_maps/HAD_Map_Service'),
                    ('~/input/path_with_lane_id', '/planning/path_with_lane_id'),
                    ('~/input/dynamic_objects', '/perception/object_recognition/objects'),
                    ('~/input/vehicle_velocity', '/vehicle/vehicle_kinematic_state'),
                    ('~/output/trajectory', 'trajectory'),
                    ('/clock', '/lgsvl/clock')
                    ],
        parameters=[
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
            },
            vehicle_constants_param,
            blind_spot_param,
            crosswalk_param,
            detection_area_param,
            intersection_param,
            stop_line_param,
        ],
    )

    return launch.LaunchDescription([behavior_velocity_planner_nodes])
