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

    # handles
    robot      = LaunchConfiguration("robot")
    comms_flag = LaunchConfiguration("comms_flag")
    fish_robot = LaunchConfiguration("fish_robot")
    joy_dev    = LaunchConfiguration("joy_dev")

    # ─────────── PEZ branch ─────────────────────────────────
    pez_params = pkg_joy_share("config", "pez_joy_config.yaml")

    pez_ns_cond = PythonExpression([
        '("', robot, '" == "pez") and (("', comms_flag, '" == "false"))' #and ("', fish_robot, '" == "true"))'
    ])

    pez_ns_group = GroupAction(
        [
            PushRosNamespace("pez"),
            Node(package="joy", executable="joy_node",
                 name="joy_node", output="screen",
                 parameters=[{"dev": joy_dev, "deadzone": 0.05}]),
            Node(package="pez_joy", executable="pez_joy",
                 name="pez_joy_node", output="screen",
                 parameters=[pez_params]),
        ],
        condition=IfCondition(pez_ns_cond),
    )

    host_ns_cond = PythonExpression([
        '("', robot, '" == "pez") and ("', comms_flag,
        '" == "true")'# and ("', fish_robot, '" == "false")'
    ])

    host_ns_group = GroupAction(
        [
            PushRosNamespace("host"),
            Node(package="joy", executable="joy_node",
                 name="joy_node", output="screen",
                 parameters=[{"dev": joy_dev, "deadzone": 0.05}]),
            Node(package="pez_joy", executable="pez_joy",
                 name="pez_joy_node", output="screen",
                 parameters=[pez_params]),
        ],
        condition=IfCondition(host_ns_cond),
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
        pez_ns_group, host_ns_group, bluerov_group,
    ])
