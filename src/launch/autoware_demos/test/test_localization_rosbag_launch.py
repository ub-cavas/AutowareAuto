# Copyright 2022 the Autoware Foundation
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

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import os
import pytest
from time import sleep
import unittest


@pytest.mark.launch_test
def generate_test_description():
    autoware_demos_share_dir = get_package_share_directory('autoware_demos')
    launch_file = 'launch/localization_rosbag.launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(autoware_demos_share_dir, launch_file)),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestAutowareReady(unittest.TestCase):
    def test_init_done(self, proc_output):
        # Wait for initialization to be complete
        sleep(5)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that all processes in the launch file exit with code 0
        # Exception: ros2 bag play exits with code 2 when it cannot find a bag file
        # Exception: rviz might exit with other codes
        for process_name in proc_info.process_names():
            if "ros2" in process_name:
                launch_testing.asserts.assertExitCodes(proc_info, [0, 2], process=process_name)
            elif "rviz" not in process_name:
                launch_testing.asserts.assertExitCodes(proc_info, process=process_name)
