from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # choose your namespace and config file at launch time
    ns     = LaunchConfiguration('namespace',   default='host')
    config = LaunchConfiguration('config_file', default=PathJoinSubstitution([
                FindPackageShare('pez_comms'), 'config', 'host_comms.yaml'
            ]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='host',
            description='Namespace under which to run the comms node'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=config,
            description='Path to YAML config for host side'
        ),

        # push the namespace for everything below
        PushRosNamespace(ns),

        # include the generic comms_launch.py, passing in the config_file arg
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('pez_comms'), 'launch', 'comms_launch.py'
                ])
            ),
            launch_arguments={'config_file': config}.items()
        ),
    ])
