# launch/test_comms.launch.py

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    ExecuteProcess,
    GroupAction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1) Launch‐time arguments
    fish_port = LaunchConfiguration('fish_port', default='/tmp/pez_fish')
    host_port = LaunchConfiguration('host_port', default='/tmp/pez_host')
    fish_ns   = LaunchConfiguration('fish_ns',   default='fish')
    host_ns   = LaunchConfiguration('host_ns',   default='host')

    fish_cfg = PathJoinSubstitution([
        FindPackageShare('pez_comms'),
        'config', 'fish_comms.yaml'
    ])
    host_cfg = PathJoinSubstitution([
        FindPackageShare('pez_comms'),
        'config', 'host_comms.yaml'
    ])

    return LaunchDescription([

        # ─── Declare Arguments ──────────────────────────────────────────
        DeclareLaunchArgument('fish_port',
            default_value=fish_port,
            description='PTT link for fish side'),
        DeclareLaunchArgument('host_port',
            default_value=host_port,
            description='PTT link for host side'),
        DeclareLaunchArgument('fish_ns',
            default_value=fish_ns,
            description='Namespace for fish comms'),
        DeclareLaunchArgument('host_ns',
            default_value=host_ns,
            description='Namespace for host comms'),

        # ─── Start socat to link the two PTYs ──────────────────────────
        ExecuteProcess(
            cmd=[
                'socat', '-d', '-d',
                f'pty,raw,echo=0,link={host_port.perform(None)}',
                f'pty,raw,echo=0,link={fish_port.perform(None)}'
            ],
            output='screen'
        ),

        # ─── Inform the user ────────────────────────────────────────────
        LogInfo(msg=[
            'test_comms: linked ',
            host_port, ' ↔ ', fish_port
        ]),

        # ─── Fish‐side group ────────────────────────────────────────────
        GroupAction([
            PushRosNamespace(fish_ns),
            Node(
                package='pez_comms',
                executable='comms',
                name='comms_fish',
                output='screen',
                parameters=[
                    {'config': fish_cfg},
                    {'port': fish_port},
                ],
            ),
        ]),

        # ─── Host‐side group ────────────────────────────────────────────
        GroupAction([
            PushRosNamespace(host_ns),
            Node(
                package='pez_comms',
                executable='comms',
                name='comms_host',
                output='screen',
                parameters=[
                    {'config': host_cfg},
                    {'port': host_port},
                ],
            ),
        ]),
    ])
