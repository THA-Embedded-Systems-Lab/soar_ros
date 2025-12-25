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
import os

import launch
import launch.actions
import launch_ros
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy

from ament_index_python.packages import get_package_share_directory


import os
from ament_index_python.packages import get_package_share_directory
from time import sleep
import subprocess

@pytest.mark.launch_test
def generate_test_description():
    """Generate the test launch description"""
    
    # Get the path to our Fibonacci server script  
    # Use current file's directory to find the server script
    current_dir = os.path.dirname(os.path.abspath(__file__))
    server_script = os.path.join(current_dir, 'fibonacci_action_server.py')
    
    # Launch the Fibonacci action server
    fibonacci_server = launch.actions.ExecuteProcess(
        cmd=['python3', server_script],
        name='fibonacci_action_server',
        output='screen'
    )

    # Launch our action client test
    action_client_test = launch_ros.actions.Node(
        package='soar_ros',
        executable='test_action_client',
        name='test_action_client',
        output='screen',
        parameters=[
            {'debug': False}
        ]
    )

    return launch.LaunchDescription([
        fibonacci_server,
        # Wait a bit for the server to start
        launch.actions.TimerAction(
            period=1.0,
            actions=[action_client_test]
        ),
        launch_testing.actions.ReadyToTest()
    ])

class TestSoarRos(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_client(self, proc_output):
        # Start the event loop via spin, otherwise the service does not react.
        message = "Final Fibonacci sequence: [0, 1, 1, 2, 3, 5]"
        proc_output.assertWaitFor(message, timeout=5)
