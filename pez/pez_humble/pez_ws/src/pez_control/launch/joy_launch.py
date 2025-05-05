# File: launch/pez_full.launch.py

import os

from launch import LaunchDescription
from launch.actions import GroupAction
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('pez_control')

    # Paths to your YAML & perspective files in <pkg>/config
    joy_params    = os.path.join(pkg_share, 'config', 'joystick_params.yaml')
    axis_params   = os.path.join(pkg_share, 'config', 'axis_params.yaml')
    rqt_persp     = os.path.join(pkg_share, 'config', 'pez.perspective')

    return LaunchDescription([
        GroupAction([
            # everything lives under /pez
            launch_ros.actions.PushRosNamespace('pez'),

            # Joy → cmd_vel publisher
            launch_ros.actions.Node(
                package='pez_control',
                executable='joy_node',
                name='joy_controller',
                output='screen',
                parameters=[joy_params],
            ),

            # RQt with saved perspective
            launch_ros.actions.Node(
                package='rqt_gui',
                executable='rqt_gui',
                name='rqt',
                output='screen',
                arguments=[
                    '--perspective-file', rqt_persp
                ],
                # if you need extra plugins, uncomment below
                # parameters=[{'plugin_paths': [...]}],
            ),
        ]),
    ])
