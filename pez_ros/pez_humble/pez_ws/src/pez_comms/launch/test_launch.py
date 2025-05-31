# pez_comms/launch/test_socat_dynamic.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1) Run socat without a fixed “link=…”—let it choose two free PTYs
        ExecuteProcess(
            cmd=['socat', '-d', '-d', 'pty,raw,echo=0', 'pty,raw,echo=0'],
            output='screen',
            shell=False
        ),

        # 2) Log a message so you remember to copy-and-paste the PTYs from socat's stdout
        LogInfo(msg=[
            '→ socat has started.  Check its console output for two PTY names (e.g. /dev/pts/7 and /dev/pts/8).'
        ]),

        # 3) (These Node entries won’t actually start until you manually re-run this launch
        #     with the PTY names you saw. We leave them here as a reminder.)
        Node(
            package='pez_comms',
            executable='host_side',
            name='host_comms',
            namespace='host',
            output='screen',
            parameters=[{'port': '/dev/pts/4'}],
            # <-- replace <copy_host_here> with the first PTY you saw
        ),
        Node(
            package='pez_comms',
            executable='fish_side',
            name='fish_comms',
            namespace='pez',
            output='screen',
            parameters=[{'port': '/dev/pts/5'}],
            # <-- replace <copy_fish_here> with the second PTY you saw
        ),
    ])
