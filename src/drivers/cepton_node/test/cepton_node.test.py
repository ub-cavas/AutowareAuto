# Copyright 2018 Apex.AI, Inc.
# All rights reserved.
import unittest

import ament_index_python
import launch
import launch.actions
import launch_ros.actions
from launch_ros.default_launch_description import ROSSpecificLaunchStartup
import rclpy.context

import launch_testing.event_handlers
import lidar_integration
import launch_testing.asserts

from apex_pytest_utils import quarantine

def generate_test_description(ready_fn):

    PORT=lidar_integration.get_open_port()

    # The node under test
    cepton_cloud_node = launch_ros.actions.LifecycleNode(
        package="cepton_node",
        node_executable="cepton_cloud_node_exe",
        node_name="cepton_cloud_node",
        parameters=[
            "{}/cepton_test.param.yaml".format(
                ament_index_python.get_package_share_directory("cepton_node")
            ),
            {
                "port": str(PORT)
            },
        ]
    )

    spoofer = launch_ros.actions.Node(
        package="cepton_node",
        node_executable="cepton_replay_exe",
        arguments=[
            "--file",
            "{}/lidar.pcap".format(
                ament_index_python.get_package_share_directory("cepton_node")
            ),
            "--port", str(PORT)
        ],
    )

    lidar_checker = lidar_integration.make_pcl_checker(
        topic="cepton_cloud0",
        size=4000,
        period=75,
        size_tolerance=0.5,
        period_tolerance=0.5,
    )

    return lidar_integration.get_lidar_launch_description(
        spoofer=spoofer,  # cepton node needs a different spoofer than the default
        test_nodes=[cepton_cloud_node],
        checkers=[lidar_checker],
        other_actions=[
            launch.actions.OpaqueFunction(function=lambda context: ready_fn())
        ],
        port=PORT,
    )


# Test cases are created automatically by the lidar_integration package.  We just need to
# instantiate them
active = lidar_integration.make_active_tests()

@launch_testing.post_shutdown_test()
class TestCheckerOutput(unittest.TestCase):

    def test_checker_outputs_success(self, _checkers):
        for checker in _checkers:
            launch_testing.asserts.assertInStdout(self.proc_output, "success", checker)

    @quarantine(4905, "10/26/2019")
    def test_spoofer_exit_code_ok(self, _spoofer):

        print("\nSpoofer output:")
        for line in self.proc_output[_spoofer]:
            print(line.text.decode(), end='')
        self.assertEqual(0, self.proc_info[_spoofer].returncode)

    @quarantine(4905, "10/26/2019")
    def test_nodes_exit_code_ok(self, _test_nodes):

        print("\nTest node outputs:")
        for nd in _test_nodes:
            for line in self.proc_output[nd]:
                print(line.text.decode(), end='')
        for node in _test_nodes:
            self.assertEqual(0, self.proc_info[node].returncode)

    def test_checker_exit_code_ok(self, _checkers):

        # Also check the exit code of the checker as a belt-and-suspenders approach
        for checker in _checkers:

            # For diagnosing problems with the test, it's helpful to have the checker output
            # for passing and failing runs
            if hasattr(checker, "_checker_id"):
                print("Checker {}".format(checker._checker_id))

            for line in self.proc_output[checker]:
                print(line.text.decode(), end='')

        # Check exit codes, don't check spoofer and actual DUT because they're dumb
        for checker in _checkers:
            self.assertEqual(0, self.proc_info[checker].returncode)

