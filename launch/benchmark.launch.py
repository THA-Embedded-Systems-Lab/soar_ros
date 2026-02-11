from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

import string

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'f',
            default_value='1',
            description='Frequency for Sender node (Hz)'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Debug mode for system node'
        ),
        DeclareLaunchArgument(
            'num_inputs',
            default_value='2',
            description='Number of input channels (for SISO/MIMO)'
        ),
        DeclareLaunchArgument(
            'num_outputs',
            default_value='2',
            description='Number of output channels (for SISO/MIMO)'
        ),
        DeclareLaunchArgument(
            'messages_to_send',
            default_value='3000',
            description='Number of messages to send from Sender node'
        ),
        DeclareLaunchArgument(
            'auto_delete_soar_io_on_complete',
            default_value='false',
            description='Enable automatic deletion of completed input/output messages'
        ),
        # Start sender with two second delay, so the receiver and system are ready
        TimerAction(
            period=0.5,
            actions=[
                Node(
                    package='soar_ros',
                    executable='Sender',
                    name='sender',
                    parameters=[
                        {'frequency': LaunchConfiguration('f')},
                        {'num_inputs': LaunchConfiguration('num_inputs')},
                        {'messages_to_send': LaunchConfiguration('messages_to_send')},
                    ]
                )
            ]
        ),
        Node(
            package='soar_ros',
            executable='Receiver',
            name='receiver',
            parameters=[
                {'num_outputs': LaunchConfiguration('num_outputs')},
            ]
        ),
        Node(
            package='soar_ros',
            executable='System',
            name='system',
            parameters=[
                {'debug': LaunchConfiguration('debug')},
                {'auto_delete_soar_io_on_complete': LaunchConfiguration('auto_delete_soar_io_on_complete')},
                {'num_inputs': LaunchConfiguration('num_inputs')},
                {'num_outputs': LaunchConfiguration('num_outputs')},
           ]
        )
    ])
