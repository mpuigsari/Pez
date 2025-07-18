# launch/teleop_launch.py
"""Joystick teleoperation groups for PEZ or host."""

import os
from launch import LaunchDescription
from launch.actions import GroupAction, PushRosNamespace
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    comms_flag = LaunchConfiguration("comms_flag")
    fish_robot = LaunchConfiguration("fish_robot")

    pkg_joy = get_package_share_directory("pez_joy")
    joy_params = os.path.join(pkg_joy, "config", "pez_joy_config.yaml")

    pez_condition = PythonExpression(
        ['"', comms_flag, '" == "false" or "', fish_robot, '" == "true"']
    )
    pez_group = GroupAction(
        actions=[
            PushRosNamespace("pez"),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[{"dev": "/dev/input/js0", "deadzone": 0.05}],
            ),
            Node(
                package="pez_joy",
                executable="pez_joy",
                name="pez_joy_node",
                output="screen",
                parameters=[joy_params],
            ),
        ],
        condition=IfCondition(pez_condition),
    )

    host_condition = PythonExpression(
        ['"', comms_flag, '" == "true" and "', fish_robot, '" == "false"']
    )
    host_group = GroupAction(
        actions=[
            PushRosNamespace("host"),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[{"dev": "/dev/input/js0", "deadzone": 0.05}],
            ),
            Node(
                package="pez_joy",
                executable="pez_joy",
                name="pez_joy_node",
                output="screen",
                parameters=[joy_params],
            ),
        ],
        condition=IfCondition(host_condition),
    )

    return LaunchDescription(
        [
            pez_group,
            host_group,
        ]
    )
