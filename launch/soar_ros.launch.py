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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "debug",
            description="Set the system in a debug mode.",
            default_value="False"
        )
    )
    debug = LaunchConfiguration("debug")

    return LaunchDescription(declared_arguments + [
        Node(
            package='soar_ros',
            executable='start',
            name='soar_ros',
            emulate_tty=True,
            output={'screen'},
            parameters=[
                {'debug': debug}
            ]
        ),
    ])
