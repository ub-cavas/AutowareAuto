#!/usr/bin/env python3

# Copyright 2022 Arm Ltd
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

from rclpy.node import Node

import lgsvl
import time


class LGSVLSimulationNode(Node):
    """Node used to run a simulation on LG SVL through the Python API."""

    def __init__(self):
        super().__init__('lgsvl_simulation_node')

        # Simulation parameters
        self.param_api_endpoint_address = self.declare_parameter('api_endpoint.address').value
        self.param_api_endpoint_port = self.declare_parameter('api_endpoint.port').value

        self.param_scene_uuid = self.declare_parameter('scene_uuid').value

        self.param_vehicle_config_uuid = self.declare_parameter('vehicle.config_uuid').value
        self.param_vehicle_spawn_position = lgsvl.geometry.Vector(
            self.declare_parameter('vehicle.spawn.position.x').value,
            self.declare_parameter('vehicle.spawn.position.y').value,
            self.declare_parameter('vehicle.spawn.position.z').value,
        )
        self.param_vehicle_spawn_rotation = lgsvl.geometry.Vector(
            self.declare_parameter('vehicle.spawn.rotation.x').value,
            self.declare_parameter('vehicle.spawn.rotation.y').value,
            self.declare_parameter('vehicle.spawn.rotation.z').value,
        )

        self.param_vehicle_bridge_address = self.declare_parameter('vehicle.bridge.address').value
        self.param_vehicle_bridge_port = self.declare_parameter('vehicle.bridge.port').value

    def setup(self):
        # Simulator setup
        self.sim = lgsvl.Simulator(
            address=self.param_api_endpoint_address,
            port=self.param_api_endpoint_port,
        )

        # Scene setup
        if self.sim.current_scene == self.param_scene_uuid:
            self.sim.reset()
        else:
            self.sim.load(scene=self.param_scene_uuid)

        # Vehicle setup
        self.vehicle_state = lgsvl.AgentState()
        self.vehicle_state.transform.position = self.param_vehicle_spawn_position
        self.vehicle_state.transform.rotation = self.param_vehicle_spawn_rotation

        self.vehicle_agent = self.sim.add_agent(
            name=self.param_vehicle_config_uuid,
            agent_type=lgsvl.AgentType.EGO,
            state=self.vehicle_state,
        )

        if not self.vehicle_agent.bridge_connected:
            self.vehicle_agent.connect_bridge(
                address=self.param_vehicle_bridge_address,
                port=self.param_vehicle_bridge_port,
            )
            self.get_logger().info('waiting for ROS2 bridge connection')

            for _ in range(5):
                if not self.vehicle_agent.bridge_connected:
                    time.sleep(1)

        if self.vehicle_agent.bridge_connected:
            self.get_logger().info('ROS2 bridge connection successful')
        else:
            raise RuntimeError('ROS2 bridge connection failed')

    def run_simulation(self):
        if hasattr(self, 'sim'):
            self.get_logger().info('starting simulation')
            self.sim.run()
        else:
            raise RuntimeError('cannot run simulation - the simulator is not setup!')

    def shutdown(self):
        if hasattr(self, 'sim'):
            self.sim.stop()
