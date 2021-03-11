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

import rclpy
from lgsvl_simulation.lgsvl_simulation_node import LGSVLSimulationNode


def main(args=None):
    rclpy.init(args=args)
    node = LGSVLSimulationNode()
    rclpy.get_default_context().on_shutdown(node.shutdown)

    try:
        node.setup()
        future = rclpy.get_global_executor().create_task(node.run_simulation)
        rclpy.spin_until_future_complete(node, future)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
