from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    cfg = LaunchConfiguration('config_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='config/bluerov_comms.yaml',
            description='Path to the comms YAML profile'
        ),
        Node(
            package='pez_comms',
            executable='comms',
            name='comms',
            output='screen',
            arguments=[ '--config-file', cfg ],
        )
    ])
