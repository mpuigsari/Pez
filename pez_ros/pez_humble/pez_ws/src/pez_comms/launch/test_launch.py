# pez_comms/launch/test_socat.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # launch arguments
    host_port = LaunchConfiguration('host_port')
    fish_port = LaunchConfiguration('fish_port')

    return LaunchDescription([
        # Arguments for the two ends of the virtual serial link
        DeclareLaunchArgument('host_port', default_value='/dev/pts/5',
                              description='PTY for host_side to open'),
        DeclareLaunchArgument('fish_port', default_value='/dev/pts/6',
                              description='PTY for fish_side to open'),

        # Launch socat to tie them together
        ExecuteProcess(
            cmd=[
                'socat', '-d', '-d',
                'pty,raw,echo=0,link=' + host_port,
                'pty,raw,echo=0,link=' + fish_port
            ],
            output='screen',
            shell=False
        ),

        # Give socat a moment to create the PTYs
        LogInfo(msg=["Waiting for PTYs: ", host_port, " <--> ", fish_port]),

        # Host-side node
        Node(
            package='pez_comms',
            executable='host_side',
            name='host_comms',
            output='screen',
            parameters=[{'port': host_port}],
        ),

        # Fish-side node
        Node(
            package='pez_comms',
            executable='fish_side',
            name='fish_comms_fish',
            output='screen',
            parameters=[{'port': fish_port}],
        ),
    ])
