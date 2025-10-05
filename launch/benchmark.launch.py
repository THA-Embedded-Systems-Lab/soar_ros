from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

import string

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'f',
            default_value='1.0',
            description='Frequency for Sender node (Hz)'
        ),
        DeclareLaunchArgument(
            'run_id',
            default_value='default',
            description='Run identifier string'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Debug mode for system node'
        ),
        Node(
            package='soar_ros',
            executable='Sender',
            name='sender',
            parameters=[
                {'frequency': LaunchConfiguration('f')},
            ]
        ),
        Node(
            package='soar_ros',
            executable='Receiver',
            name='receiver',
        ),
        Node(
            package='soar_ros',
            executable='System',
            name='system',
            parameters=[
                {'debug': LaunchConfiguration('debug')},
           ]
        )
    ])
