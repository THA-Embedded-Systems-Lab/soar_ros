# Copyright 2024 Moritz Schmidt
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

import unittest

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest

import rclpy

import std_msgs.msg
import example_interfaces.srv
import threading

# Constants for timeouts
SERVICE_TIMEOUT = 1.0


@pytest.mark.launch_test
def generate_test_description():
    soar_ros_node = launch_ros.actions.Node(
        package="soar_ros",
        executable="test_service",
        shell=True,
        emulate_tty=True,
        output='screen'
    )

    return launch.LaunchDescription([
        soar_ros_node,
        launch_testing.actions.ReadyToTest()
    ])


def spin_node(node, stop_event):
    while not stop_event.is_set():
        rclpy.spin_once(node)


class TestSoarRos(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_soar_ros')

    def tearDown(self):
        self.node.destroy_node()


    def test_service(self, proc_output):
        cli = self.node.create_client(
            example_interfaces.srv.AddTwoInts, "AddTwoInts")

        if not cli.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.node.get_logger().error('Service not available after waiting.')
            self.fail('Service not available')

        req = example_interfaces.srv.AddTwoInts.Request()
        req.a = 2
        req.b = 2

        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is None:
            self.node.get_logger().error(f'Failed to call service: {future.exception()}')
            self.fail('Service call failed')

        self.node.get_logger().info(f'Service call successful: {future.result()}')
        self.assertEqual(future.result().sum, 4)
