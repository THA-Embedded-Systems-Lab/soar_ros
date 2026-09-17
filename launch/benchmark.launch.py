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
            'preferences_based_sort',
            default_value='False',
            description='Enable preferences-based sorting in Soar System node'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Debug mode for system node'
        ),
        DeclareLaunchArgument(
            'num_inputs',
            default_value='1',
            description='Number of input channels (for SISO/MIMO)'
        ),
        DeclareLaunchArgument(
            'num_outputs',
            default_value='1',
            description='Number of output channels (for SISO/MIMO)'
        ),
        DeclareLaunchArgument(
            'messages_to_send',
            default_value='5',
            description='Number of messages to send from Sender node'
        ),
        DeclareLaunchArgument(
            'auto_delete_soar_io_on_complete',
            default_value='True',
            description='Enable automatic deletion of completed input/output messages'
        ),
        DeclareLaunchArgument(
            'staircase_mode',
            default_value='False',
            description='Enable staircase frequency ramp in Sender node instead of a fixed frequency'
        ),
        DeclareLaunchArgument(
            'staircase_start_freq',
            default_value='1000',
            description='Staircase mode: starting frequency (Hz)'
        ),
        DeclareLaunchArgument(
            'staircase_end_freq',
            default_value='3000',
            description='Staircase mode: final frequency (Hz)'
        ),
        DeclareLaunchArgument(
            'staircase_step_freq',
            default_value='200',
            description='Staircase mode: frequency increment per step (Hz)'
        ),
        DeclareLaunchArgument(
            'staircase_hold_duration',
            default_value='4.0',
            description='Staircase mode: duration to hold each frequency step (s)'
        ),
        DeclareLaunchArgument(
            'staircase_recovery_duration',
            default_value='1.0',
            description='Staircase mode: recovery pause between frequency steps (s)'
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
                        {'staircase_mode': LaunchConfiguration('staircase_mode')},
                        {'staircase_start_freq': LaunchConfiguration('staircase_start_freq')},
                        {'staircase_end_freq': LaunchConfiguration('staircase_end_freq')},
                        {'staircase_step_freq': LaunchConfiguration('staircase_step_freq')},
                        {'staircase_hold_duration': LaunchConfiguration('staircase_hold_duration')},
                        {'staircase_recovery_duration': LaunchConfiguration('staircase_recovery_duration')},
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
                {'preferences_based_sort': LaunchConfiguration('preferences_based_sort')},
                {'num_inputs': LaunchConfiguration('num_inputs')},
                {'num_outputs': LaunchConfiguration('num_outputs')},
           ]
        )
    ])
