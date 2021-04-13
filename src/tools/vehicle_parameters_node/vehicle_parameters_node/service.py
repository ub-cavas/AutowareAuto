# Copyright 2021 Autoware Foundation, Inc.
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

import rclpy
from rclpy.node import Node


class VehicleParametersNode(Node):

    def __init__(self):
        super().__init__('vehicle_parameters')


def main(args=None):
    rclpy.init(args=args)

    vehicle_parameters_node = VehicleParametersNode()

    rclpy.spin(vehicle_parameters_node)

    rclpy.shutdown()
