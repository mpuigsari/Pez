#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ─────── Launch arguments ────────────────────────────────────────────
    script_arg = DeclareLaunchArgument(
        "script",
        default_value=PathJoinSubstitution(
            [FindPackageShare("pez_joy"), "config", "pez_experiment.yaml"]
        ),
        description="YAML file that drives the player",
    )

    rate_arg = DeclareLaunchArgument(
        "rate_hz",
        default_value="20.0",
        description="cmd_vel publish rate (Hz)",
    )

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="joy",
        choices=["cmd", "joy"],
        description='Which executable to launch: "cmd" = command_player, "joy" = joy_player',
    )
    timer_arg = DeclareLaunchArgument(
        "timer",
        default_value="5.0",
        description="Timer frequency for joy_player (Hz, default=4.0)",
    )
    

    # ─────── Common substitutions ────────────────────────────────────────
    yaml_file = LaunchConfiguration("script")
    rate_hz   = LaunchConfiguration("rate_hz")
    mode      = LaunchConfiguration("mode")
    timer_sec = LaunchConfiguration("timer")
    """ns_arg = DeclareLaunchArgument(
        "namespace",
        default_value="pez",
        description="ROS namespace for the pez robot (ignored for cmd_player)",
    )
    namespace = LaunchConfiguration("namespace")"""

    # ─────── Nodes with conditions ───────────────────────────────────────

    cmd_player_node = Node(
        package="pez_joy",
        executable="command_player",
        name="command_player",
        output="screen",
        arguments=["-f", yaml_file, "-r", rate_hz],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'cmd'"])),
    )

    joy_player_node = Node(
        package="pez_joy",
        executable="joy_player",
        name="joy_player",
        output="screen",
        parameters=[{"timer_sec": timer_sec}],              # ← aquí
        condition=IfCondition(PythonExpression(["'", mode, "' == 'joy'"])),
    )
    namespace_group = GroupAction(
        [
            #PushRosNamespace(namespace),
            cmd_player_node,
            joy_player_node,
        ]
    )

    # optional: fix a domain-ID for multi-robot sims
    set_domain = SetEnvironmentVariable("ROS_DOMAIN_ID", "21")

    # ─────── LaunchDescription ───────────────────────────────────────────
    return LaunchDescription(
        [
            # set_domain,                 # uncomment if you really need it
            script_arg,
            rate_arg,
            mode_arg,
            timer_arg,
            #ns_arg,
            namespace_group,
        ]
    )
