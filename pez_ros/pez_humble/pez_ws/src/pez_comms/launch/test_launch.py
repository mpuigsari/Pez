# launch/test_launch.py

from launch import LaunchDescription
from launch.actions import LogInfo, ExecuteProcess, TimerAction, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    fish_port = '/tmp/pez_fish'
    host_port = '/tmp/pez_host'
    fish_ns   = 'pez'
    host_ns   = 'host'

    share = FindPackageShare('pez_comms')
    fish_cfg = PathJoinSubstitution([share, 'config', 'fish_comms.yaml'])
    host_cfg = PathJoinSubstitution([share, 'config', 'host_comms.yaml'])

    
    socat = ExecuteProcess(
            cmd=[
                'socat','-d','-d',
                f'pty,raw,echo=0,link={host_port}',
                f'pty,raw,echo=0,link={fish_port}',
            ],
            output='screen'
        )
    socat_log = LogInfo(msg=[f'Linked {host_port} ↔ {fish_port}'])
    
    return LaunchDescription([

        #socat
        #socat_log

        # 2) wait for PTYs, then launch both
        TimerAction(
            period=1.0,
            actions=[

                GroupAction([
                    PushRosNamespace(fish_ns),
                    Node(
                        package='pez_comms',
                        executable='comms',
                        name='comms_fish',
                        output='screen',
                        arguments=[ '--config-file', fish_cfg ],
                    ),
                ]),

                GroupAction([
                    PushRosNamespace(host_ns),
                    Node(
                        package='pez_comms',
                        executable='comms',
                        name='comms_host',
                        output='screen',
                        arguments=[ '--config-file', host_cfg ],
                    ),
                ]),
            ]
        ),
    ])
