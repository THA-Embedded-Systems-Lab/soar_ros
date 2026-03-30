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

# Test: two agents subscribed to the same ROS 2 topic both receive the
# message on their Soar input links and independently echo it to their
# output links.

import unittest
import uuid

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import pytest

import rclpy
import std_msgs.msg

from time import sleep


@pytest.mark.launch_test
def generate_test_description():
    soar_ros_node = launch_ros.actions.Node(
        package="soar_ros",
        executable="test_shared_input",
        shell=True,
        emulate_tty=True,
        output='screen'
    )

    return launch.LaunchDescription([
        soar_ros_node,
        launch_testing.actions.ReadyToTest()
    ])


class TestSharedInputMultiAgent(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_shared_input_soar_ros')

    def tearDown(self):
        self.node.destroy_node()

    def test_both_agents_receive_shared_input(self, proc_output):
        """Both agents must independently write the received data to their
        output links when a single message is published to the shared topic."""
        pub = self.node.create_publisher(
            std_msgs.msg.String,
            "shared_input",
            10
        )

        msg = std_msgs.msg.String()
        msg.data = str(uuid.uuid4())

        sleep(1)
        pub.publish(msg)

        # Agent A must have received and echoed the message.
        proc_output.assertWaitFor(
            f"SHARED_ECHO_A:{msg.data}", timeout=5, stream='stdout')

        # Agent B must have received and echoed the message independently.
        proc_output.assertWaitFor(
            f"SHARED_ECHO_B:{msg.data}", timeout=5, stream='stdout')
