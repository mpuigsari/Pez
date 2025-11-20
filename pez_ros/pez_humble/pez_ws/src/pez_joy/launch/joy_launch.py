#!/usr/bin/env python3
"""
Unified tele-operation launch for

  • PEZ robot          (`robot:=pez`)      – keeps the old comms / fish flags
  • BlueROV derivative (`robot:=bluerov`)  – simpler, fixed namespace
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
)
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


# ───────────────────────── helper ──────────────────────────
def pkg_joy_share(*parts):
    return os.path.join(get_package_share_directory("pez_joy"), *parts)


# ──────────────────────── launch file ──────────────────────
def generate_launch_description() -> LaunchDescription:

    # ─────────── top-level arguments ───────────────────────
    robot_arg = DeclareLaunchArgument(
        "robot", default_value="pez",
        description='Robot type: "pez" (default) or "bluerov".',
    )
    comms_arg = DeclareLaunchArgument(
        "comms_flag", default_value="false",
        description="Enable pez_comms bridge (true/false).",
    )
    fish_arg = DeclareLaunchArgument(
        "fish_robot", default_value="true",
        description="true → running on the robot, false → host-side tele-op.",
    )
    joy_arg = DeclareLaunchArgument(
        "joy_dev", default_value="/dev/input/js0",
        description="Joystick device path.",
    )
    ns_arg = DeclareLaunchArgument(
        "namespace", default_value="bluerov",
        description="ROS namespace for BlueROV branch (ignored for PEZ).",
    )
    teleop_cfg_arg = DeclareLaunchArgument(
        "teleop_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("pez_joy"), "config", "bluerov_joy_config.yaml"]
        ),
        description="BlueROV joystick-mapping YAML.",
    )
    experiment_arg = DeclareLaunchArgument(
        "experiment",
        default_value="false",
        choices=["false","cmd", "joy"],
        description='Which executable to launch: "cmd" = command_player, "joy" = joy_player',
    )

    # handles
    robot      = LaunchConfiguration("robot")
    comms_flag = LaunchConfiguration("comms_flag")
    joy_dev    = LaunchConfiguration("joy_dev")
    experiment = LaunchConfiguration("experiment")


    # ─────────── PEZ branch ─────────────────────────────────
    pez_params = pkg_joy_share("config", "pez_joy_config.yaml")

    # ---------- el namespace que toca ----------
    # pez  + !comms   => "pez"
    # pez  +  comms   => "host"
    # bluerov         => "bluerov"   (fuera de este grupo)
    ns_name = PythonExpression([
    '"bluerov" if "', robot, '" == "bluerov" else ('
    '"host" if "', comms_flag, '" == "true" else "pez")'
])

    pez_group = GroupAction(
        [
            # El namespace cambia según el modo
            PushRosNamespace(ns_name),

            # 1) Launch JOY de siempre (si experiment==false)
            Node(
                package="joy", executable="joy_node",
                name="joy_node", output="screen",
                parameters=[{"dev": joy_dev, "deadzone": 0.05}],
                condition=IfCondition(PythonExpression(["'", experiment, "' == 'false'"]))
            ),

            # 2) Launch experimental (si experiment==true)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("pez_joy"), "launch", "experiment_launch.py"])
                ),
                launch_arguments={
                    "namespace": ns_name,
                    "mode": experiment,
                }.items(),
                condition=IfCondition(PythonExpression(["'", experiment, "' != 'false'"]))
            ),
            Node(
                package="pez_joy", executable="pez_joy",
                name="pez_joy_node", output="screen",
                parameters=[ pez_params ],
            ),
        ],
        # Sólo si robot == pez   (para bluerov usarías otro group)
        condition=IfCondition(PythonExpression(['"', robot, '" == "pez"'])),
    )

    # ─────────── BlueROV branch ─────────────────────────────
    bluerov_cond = PythonExpression(['"', robot, '" == "bluerov"'])

    bluerov_group = GroupAction(
        [
            PushRosNamespace(LaunchConfiguration("namespace")),
            Node(package="joy", executable="joy_node",
                 name="joy_node", output="screen",
                 parameters=[{"dev": joy_dev}]),
            Node(package="pez_joy", executable="bluerov_joy",
                 name="bluerov_joy", output="screen",
                 arguments=["-c", LaunchConfiguration("teleop_config")]),
        ],
        condition=IfCondition(bluerov_cond),
    )

    # ─────────── assemble ──────────────────────────────────
    return LaunchDescription([
        robot_arg, comms_arg, fish_arg,
        joy_arg, ns_arg, teleop_cfg_arg,
        experiment_arg,
        pez_group, bluerov_group,
    ])
