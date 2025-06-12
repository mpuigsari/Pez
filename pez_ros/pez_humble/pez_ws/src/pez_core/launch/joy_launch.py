# File: src/pez_core/launch/joy_launch.py

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ——————————————————————————————————————————————————
    # 1) Arguments & env
    # ——————————————————————————————————————————————————
    display_flag  = LaunchConfiguration('display_flag')
    fish_robot    = LaunchConfiguration('fish_robot')
    comms_flag    = LaunchConfiguration('comms_flag')

    # force domain
    set_domain = SetEnvironmentVariable('ROS_DOMAIN_ID', '21')

    # declare flags
    declare_display = DeclareLaunchArgument(
        'display_flag', default_value='true',
        description='If true, launch PlotJuggler & RQT'
    )
    declare_fish_robot = DeclareLaunchArgument(
        'fish_robot', default_value='true',
        description='If false, run host-side teleop instead of on-robot controller'
    )
    declare_comms = DeclareLaunchArgument(
        'comms_flag', default_value='false',
        description='If true, launch the pez_comms bridge'
    )

    # ——————————————————————————————————————————————————
    # 2) Core teleop under /pez
    # ——————————————————————————————————————————————————
    pkg_core = get_package_share_directory('pez_core')
    joy_params = os.path.join(pkg_core, 'config', 'joystick_params.yaml')

    pez_group = GroupAction([
        PushRosNamespace('pez'),
        # joystick driver
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
            }],
        ),
        # fish_joy converter (joy→/pez/cmd_vel)
        Node(
            package='pez_core',
            executable='fish_joy',
            name='fish_joy_node',
            output='screen',
            parameters=[joy_params],
        ),
    ])

    # ——————————————————————————————————————————————————
    # 3) Host-side teleop (only if fish_robot==false)
    # ——————————————————————————————————————————————————
    teleop_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_core, 'launch', 'teleop_launch.py')
        ),
        launch_arguments={'test_flag': 'true'}.items(),
        condition=UnlessCondition(fish_robot)
    )

    # ——————————————————————————————————————————————————
    # 4) Comms bridge (test or host) when comms_flag==true
    # ——————————————————————————————————————————————————
    pkg_comms = get_package_share_directory('pez_comms')
    test_launch   = os.path.join(pkg_comms, 'launch', 'test_launch.py')
    host_launch   = os.path.join(pkg_comms, 'launch', 'host_launch.py')
    host_cfg      = os.path.join(pkg_comms, 'config', 'host_comms.yaml')
    host_ns       = 'host'

    # 4a) Full test bridge (both pez+host via socat) if comms_flag && !fish_robot
    test_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(test_launch),
        condition=IfCondition(
            PythonExpression([
                "'", comms_flag, "' == 'true' and '", fish_robot, "' == 'false'"
            ])
        )
    )

    # 4b) Hardware host-only bridge if comms_flag && fish_robot
    host_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(host_launch),
        launch_arguments={
            'config_file': host_cfg,
            'namespace':    host_ns,
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'", comms_flag, "' == 'true' and '", fish_robot, "' == 'true'"
            ])
        )
    )

    # ——————————————————————————————————————————————————
    # 5) RQT & PlotJuggler
    # ——————————————————————————————————————————————————
    perspective_file = os.path.join(pkg_core, 'config', 'pez.perspective')
    layout_file      = os.path.join(pkg_core, 'config', 'test.xml')

    rqt = ExecuteProcess(
        cmd=['rqt', '--perspective-file', perspective_file],
        output='screen',
        condition=IfCondition(display_flag),
    )
    pj  = ExecuteProcess(
        cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler', '--layout', layout_file],
        output='screen',
        condition=IfCondition(display_flag),
    )

    # ——————————————————————————————————————————————————
    # 6) Assemble the launch description
    # ——————————————————————————————————————————————————
    return LaunchDescription([
        set_domain,
        declare_display,
        declare_fish_robot,
        declare_comms,

        pez_group,
        teleop_include,

        test_bridge,
        host_bridge,

        rqt,
        pj,
    ])
