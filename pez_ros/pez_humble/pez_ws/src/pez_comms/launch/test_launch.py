# launch/test_launch.py
from launch import LaunchDescription
from launch.actions import LogInfo, ExecuteProcess, TimerAction, GroupAction, DeclareLaunchArgument
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, FindExecutable

def generate_launch_description():
    fish_port = '/tmp/pez_fish'
    host_port = '/tmp/pez_host'
    fish_ns   = 'pez'
    host_ns   = 'host'
    
    share = FindPackageShare('pez_comms')
    fish_cfg = PathJoinSubstitution([share, 'config', 'fish_comms_test.yaml'])
    host_cfg = PathJoinSubstitution([share, 'config', 'host_comms_test.yaml'])
    emulator_script = PathJoinSubstitution([share, 'scripts', 'serial_emulator.py'])
    
    # Argumentos de emulación
    delay_arg = DeclareLaunchArgument(
        'delay', default_value='3000',
        description='Network delay in milliseconds (e.g., 50)'
    )
    jitter_arg = DeclareLaunchArgument(
        'jitter', default_value='500',
        description='Network jitter in milliseconds (e.g., 10)'
    )
    loss_arg = DeclareLaunchArgument(
        'loss', default_value='5',
        description='Packet loss percentage (e.g., 5 for 5%)'
    )
    
    delay = LaunchConfiguration('delay')
    jitter = LaunchConfiguration('jitter')
    loss = LaunchConfiguration('loss')
    
    # Emulador que crea y maneja ambos PTYs
    emulator = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'),
            emulator_script,
            fish_port,
            host_port,
            '--delay', delay,
            '--jitter', jitter,
            '--loss', loss,
        ],
        output='screen',
        name='serial_emulator'
    )
    
    emulator_log = LogInfo(
        msg=[f'Serial emulation: {host_port} ↔ {fish_port} ',
             f'(delay=', delay, 'ms, jitter=', jitter, 'ms, loss=', loss, '%)']
    )
    
    return LaunchDescription([
        delay_arg,
        jitter_arg,
        loss_arg,
        
        emulator,
        emulator_log,
        
        # Esperar a que los PTYs estén creados
        TimerAction(
            period=1.5,
            actions=[
                GroupAction([
                    PushRosNamespace(fish_ns),
                    Node(
                        package='pez_comms',
                        executable='comms',
                        name='comms_fish',
                        output='screen',
                        arguments=['--config-file', fish_cfg],
                    ),
                ]),
                GroupAction([
                    PushRosNamespace(host_ns),
                    Node(
                        package='pez_comms',
                        executable='comms',
                        name='comms_host',
                        output='screen',
                        arguments=['--config-file', host_cfg],
                    ),
                ]),
            ]
        ),
    ])