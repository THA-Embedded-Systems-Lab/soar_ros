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
import pytest

import rclpy

import std_msgs.msg

from time import sleep


@pytest.mark.launch_test
def generate_test_description():
    soar_ros_node = launch_ros.actions.Node(
        package="soar_ros",
        executable="test_multi_agent",
        shell=True,
        emulate_tty=True,
        output='screen'
    )

    return launch.LaunchDescription([
        soar_ros_node,
        launch_testing.actions.ReadyToTest()
    ])


class TestMultiAgentSoarRos(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_multi_agent_soar_ros')

    def tearDown(self):
        self.node.destroy_node()

    def test_two_agents(self, proc_output):
        pub_a = self.node.create_publisher(
            std_msgs.msg.String,
            "agent_a_input",
            10
        )
        pub_b = self.node.create_publisher(
            std_msgs.msg.String,
            "agent_b_input",
            10
        )

        msg_a = std_msgs.msg.String()
        msg_b = std_msgs.msg.String()
        msg_a.data = str(uuid.uuid4())
        msg_b.data = str(uuid.uuid4())

        sleep(1)
        pub_a.publish(msg_a)
        pub_b.publish(msg_b)

        proc_output.assertWaitFor(f"AGENT_A_OUTPUT:{msg_a.data}", timeout=5, stream='stdout')
        proc_output.assertWaitFor(f"AGENT_B_OUTPUT:{msg_b.data}", timeout=5, stream='stdout')
