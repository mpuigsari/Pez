# launch/fish_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1) Fixed namespace and port for fish side
    ns    = LaunchConfiguration('namespace',  default='fish')
    port  = LaunchConfiguration('port',       default='/tmp/pez_fish')
    config= PathJoinSubstitution([
                FindPackageShare('pez_comms'),
                'config', 'fish_comms.yaml'
            ])

    return LaunchDescription([
        # 2) Declare them so users can override if they really want
        DeclareLaunchArgument('namespace',
            default_value=ns,
            description='Namespace for fish-side comms'),
        DeclareLaunchArgument('config_file',
            default_value=config,
            description='Path to fish_comms.yaml'),
        DeclareLaunchArgument('port',
            default_value=port,
            description='Serial port for fish-side (e.g. /tmp/pez_fish)'),

        # 3) Push the fish namespace
        PushRosNamespace(ns),

        # 4) Launch the generic comms node, passing the YAML + override
        Node(
            package='pez_comms',
            executable='comms',
            name='comms_fish',
            output='screen',
            arguments=[ '--config-file', config ],

        ),
    ])
