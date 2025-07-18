#!/usr/bin/env python3
from datetime import datetime
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, GroupAction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    # ───────── timestamp para nombres únicos ─────────
    ts = datetime.now().strftime('%Y%m%d_%H%M%S_')   # p.ej. 20250718_214550_

    # ───────── argumentos de usuario ────────────────
    robot_arg   = DeclareLaunchArgument('robot',      default_value='pez')
    comms_arg   = DeclareLaunchArgument('comms_flag', default_value='false')

    yaml_arg    = DeclareLaunchArgument(
        'script',
        default_value=PathJoinSubstitution([
            FindPackageShare('pez_joy'), 'config', 'pez_experiment.yaml'
        ]),
    )

    bag_host_arg = DeclareLaunchArgument(
        'bag_host_cable',
        default_value=TextSubstitution(text=f'{ts}bag_host_cable')
    )
    bag_pez_arg  = DeclareLaunchArgument(
        'bag_pez_comms',
        default_value=TextSubstitution(text=f'{ts}bag_pez_comms')
    )

    # handles
    comms_flag   = LaunchConfiguration('comms_flag')
    bag_host_out = LaunchConfiguration('bag_host_cable')
    bag_pez_out  = LaunchConfiguration('bag_pez_comms')
    yaml_file    = LaunchConfiguration('script')

    # ───────── bag DOMAIN 21 ─────────────────────────
    bag_host_group = GroupAction([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '21'),
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '/host/cmd_vel',
                '/pez/cmd_vel', '/pez/pwm',
                '--regex', '/pez/(tsys01|ms5837)/.*',
                '-o', bag_host_out,
            ],
            output='screen',
        ),
    ])

    # ───────── bag DOMAIN 22 (solo si comms_flag==true) ──
    bag_pez_group = GroupAction(
        actions=[
            SetEnvironmentVariable('ROS_DOMAIN_ID', '22'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record',
                    '/pez/cmd_vel', '/pez/pwm',
                    '--regex', '/pez/(tsys01|ms5837)/.*',
                    '-o', bag_pez_out,
                ],
                output='screen',
            ),
        ],
        condition=IfCondition(PythonExpression(['"', comms_flag, '" == "true"'])),
    )

    # ───────── command-player ────────────────────────
    player = Node(
        package='pez_joy',
        executable='command_player',
        name='command_player',
        output='screen',
        arguments=['-f', yaml_file],
    )

    # ───────── ensamblar LaunchDescription ───────────
    return LaunchDescription([
        robot_arg, comms_arg, yaml_arg,
        bag_host_arg, bag_pez_arg,
        bag_host_group,
        bag_pez_group,
        player,
    ])
