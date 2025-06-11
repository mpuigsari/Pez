# launch/host_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1) Fixed namespace and port for host side
    ns    = LaunchConfiguration('namespace',  default='host')
    port  = LaunchConfiguration('port',       default='/tmp/pez_host')
    config= PathJoinSubstitution([
                FindPackageShare('pez_comms'),
                'config', 'host_comms.yaml'
            ])

    return LaunchDescription([
        # 2) Declare them so users can override if they really want
        DeclareLaunchArgument('namespace',
            default_value=ns,
            description='Namespace for host-side comms'),
        DeclareLaunchArgument('port',
            default_value=port,
            description='Serial port for host-side (e.g. /tmp/pez_host)'),
        DeclareLaunchArgument('config_file',
            default_value=config,
            description='Path to host_comms.yaml'),

        # 3) Push the host namespace
        PushRosNamespace(ns),

        # 4) Launch the generic comms node, passing the YAML + override
        Node(
            package='pez_comms',
            executable='comms',
            name='comms_host',
            output='screen',
            arguments=[ '--config-file', config ],

        ),
    ])
