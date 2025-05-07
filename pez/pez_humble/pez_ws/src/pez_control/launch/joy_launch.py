# File: launch/pez_full.launch.py

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, SetEnvironmentVariable
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('pez_control')

    # Paths to your YAML & perspective files in <pkg>/config
    joy_params       = os.path.join(pkg_share, 'config', 'joystick_params.yaml')
    perspective_file = os.path.join(pkg_share, 'config', 'pez.perspective')

    return LaunchDescription([
        # 0) Force all nodes in this launch to use ROS_DOMAIN_ID=21
        SetEnvironmentVariable('ROS_DOMAIN_ID', '21'),
        # 1) Your existing /pez namespace group
        GroupAction([
            launch_ros.actions.PushRosNamespace('pez'),

            # joy_node from the joy package
            launch_ros.actions.Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen',
                parameters=[{
                    'dev': '/dev/input/js0',
                    'deadzone': 0.05,
                }]
            ),

            # Joy → cmd_vel publisher
            launch_ros.actions.Node(
                package='pez_control',
                executable='pez_joy',
                name='joy_controller',
                output='screen',
                parameters=[joy_params],
            ),
        ]),

        # 2) Start rqt with your saved perspective
        ExecuteProcess(
            cmd=[
                'rqt',
                '--perspective-file', perspective_file
            ],
            output='screen',
            # ensure rqt runs after /pez namespace is up
            shell=False
        ),
    ])

