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
import uuid

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest

import rclpy

import std_msgs.msg
import example_interfaces.srv


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


class TestSoaos(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_soar_ros')
        # The service is required by the test setup to be available. Create it,
        # but callback does nothing.
        self.srv = self.node.create_service(
            example_interfaces.srv.AddTwoInts, "AddTwoIntsClient", self.add_two_ints_callback)

    def tearDown(self):
        self.node.destroy_node()

    def test_republisher(self, proc_output):
        pub = self.node.create_publisher(
            std_msgs.msg.String,
            "testinput",
            10
        )

        msg = std_msgs.msg.String()
        msg.data = str(uuid.uuid4())
        for _ in range(10):
            pub.publish(msg)

        proc_output.assertWaitFor(msg.data, timeout=2, stream='stdout')

    def test_service(self, proc_output):
        cli = self.node.create_client(
            example_interfaces.srv.AddTwoInts, "AddTwoInts")
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = example_interfaces.srv.AddTwoInts.Request()
        req.a = 2
        req.b = 2
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        proc_output.assertWaitFor("Sum=4", timeout=10, stream='stdout')

    def add_two_ints_callback(self, request, response):
        pass
