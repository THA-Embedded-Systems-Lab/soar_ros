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

from time import sleep
import unittest

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import pytest

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import std_msgs.msg
import example_interfaces.srv
import threading

SERVICE_TIMEOUT = 1.0
PROC_OUTPUT_TIMEOUT = 5


@pytest.mark.launch_test
def generate_test_description():
    soar_ros_node = launch_ros.actions.Node(
        package="soar_ros",
        executable="test_client",
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

        # Create separate callback groups
        self.service_callback_group = ReentrantCallbackGroup()
        self.publisher_callback_group = ReentrantCallbackGroup()

        # Assign the service to its callback group
        self.srv = self.node.create_service(
            example_interfaces.srv.AddTwoInts, 
            "AddTwoIntsClient", 
            self.add_two_ints_callback,
            callback_group=self.service_callback_group
        )

        # Use a MultiThreadedExecutor
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.executor.shutdown()
        self.node.destroy_node()

    def spin_node(self):
        self.executor.spin()

    def test_client(self, proc_output):
        pub = self.node.create_publisher(
            std_msgs.msg.String, "Trigger", 10, callback_group=self.publisher_callback_group
        )
        msg = std_msgs.msg.String()
        msg.data = "start"

        # Start the event loop via spin, otherwise the service does not react.
        self.spin_thread = threading.Thread(target=self.spin_node)
        self.spin_thread.start()

        # Wait for the soar_ros node to be ready
        sleep(1)

        pub.publish(msg)
        self.node.get_logger().info(f'Published Trigger message: {msg.data}')

        proc_output.assertWaitFor("Result: 12", timeout=PROC_OUTPUT_TIMEOUT)

        # Stop spin thread and wait until stopped until further tests are executed.
        self.executor.shutdown()
        self.spin_thread.join()

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.node.get_logger().info(
            f"AddTwoIntsClient a: {request.a} b: {request.b} = {response.sum}")
        return response