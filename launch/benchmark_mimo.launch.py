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
        DeclareLaunchArgument(
            'num_inputs',
            default_value='3',
            description='Number of input channels (for SISO/MIMO)'
        ),
        DeclareLaunchArgument(
            'num_outputs',
            default_value='3',
            description='Number of output channels (for SISO/MIMO)'
        ),
        Node(
            package='soar_ros',
            executable='SenderMIMO',
            name='sender_mimo',
            parameters=[
                {'frequency': LaunchConfiguration('f')},
                {'num_inputs': LaunchConfiguration('num_inputs')},
            ]
        ),
        Node(
            package='soar_ros',
            executable='ReceiverMIMO',
            name='receiver_mimo',
            parameters=[
                {'num_outputs': LaunchConfiguration('num_outputs')},
            ]
        ),
        Node(
            package='soar_ros',
            executable='SystemMIMO',
            name='system_mimo',
            parameters=[
                {'debug': LaunchConfiguration('debug')},
                {'num_inputs': LaunchConfiguration('num_inputs')},
                {'num_outputs': LaunchConfiguration('num_outputs')},
           ]
        )
    ])
