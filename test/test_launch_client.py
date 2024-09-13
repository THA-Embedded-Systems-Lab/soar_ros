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


@pytest.mark.launch_test
def generate_test_description():
    soar_ros_node = launch_ros.actions.Node(
        package="soar_ros",
        executable="test_example",
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
        self.srv = self.node.create_service(
            example_interfaces.srv.AddTwoInts, "AddTwoIntsClient", self.add_two_ints_callback)

    def tearDown(self):
        self.node.destroy_node()

    def test_client(self, proc_output):
        pub = self.node.create_publisher(std_msgs.msg.String, "Trigger", 10)
        msg = std_msgs.msg.String()
        msg.data = "start"

        # Start the event loop via spin, otherwise the service does not react.
        self.stop_event = threading.Event()
        self.spin_thread = threading.Thread(
            target=spin_node, args=(self.node, self.stop_event))
        self.spin_thread.start()

        for _ in range(10):
            pub.publish(msg)
            print(f'Published Trigger message: {msg.data}')

        proc_output.assertWaitFor("12", timeout=2)

        # Stop spin thread and wait until stopped until further tests are executed.
        self.stop_event.set()
        self.spin_thread.join()

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.node.get_logger().info(
            f"AddTwoIntsClient a: {request.a} b: {request.b} = {response.sum}")
        return response
