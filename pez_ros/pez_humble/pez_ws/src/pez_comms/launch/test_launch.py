# pez_comms/launch/test_socat_fixed.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1) Run socat but force it to create two symlinks under /tmp
        ExecuteProcess(
            cmd=[
              'socat', '-d', '-d',
              'pty,raw,echo=0,link=/tmp/pez_host',
              'pty,raw,echo=0,link=/tmp/pez_fish'
            ],
            output='screen',
            shell=False
        ),

        # 2) Log so you can see that /tmp/pez_host & /tmp/pez_fish are now ready
        LogInfo(msg=[
            '→ socat has started.  /tmp/pez_host ↔ /tmp/pez_fish are now available.'
        ]),

        # 3) Launch host_side & fish_side pointing at the fixed symlinks
        Node(
            package='pez_comms',
            executable='host_side',
            name='host_comms',
            namespace='host',
            output='screen',
            parameters=[{'port': '/tmp/pez_host'}]
        ),
        Node(
            package='pez_comms',
            executable='fish_side',
            name='fish_comms',
            namespace='pez',
            output='screen',
            parameters=[{'port': '/tmp/pez_fish'}]
        ),
    ])
