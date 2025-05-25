# File: launch/joy_launch.py

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('pez_core')

    # Paths to your YAML & perspective files in <pkg>/config
    joy_params       = os.path.join(pkg_share, 'config', 'joystick_params.yaml')
    perspective_file = os.path.join(pkg_share, 'config', 'pez.perspective')
    layout_file = os.path.join(pkg_share, 'config', 'test.xml')
    # 3) Declare display_flag argument
    display_flag = LaunchConfiguration('display_flag')
    fish_robot = LaunchConfiguration('fish_robot')

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
                package='pez_core',
                executable='fish_joy',
                name='fish_joy_node',
                output='screen',
                parameters=[joy_params],
            ),

        ]),
        DeclareLaunchArgument(
        'display_flag',
        default_value='true',
        description='If true, launch PlotJuggler & RQT'
        ),
        DeclareLaunchArgument(
        'fish_robot',
        default_value='true',
        description='If false, run the host‐side teleop instead of the on‐robot controller'
        ),

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'teleop_launch.py')
        ),
        launch_arguments={'test_flag': 'true'}.items(),
        condition=UnlessCondition(fish_robot)
    ),

        # 2) Start rqt with your saved perspective
        ExecuteProcess(
            cmd=[
                'rqt',
                '--perspective-file', perspective_file
            ],
            output='screen',
            # ensure rqt runs after /pez namespace is up
            shell=False,
            condition=IfCondition(display_flag),
        ),
        ExecuteProcess(
        cmd=[
            'ros2', 'run', 'plotjuggler', 'plotjuggler','--layout', layout_file
        ],
        output='screen',
        condition=IfCondition(display_flag),
    )
    ])

