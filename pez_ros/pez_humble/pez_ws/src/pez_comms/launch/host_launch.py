#!/usr/bin/env python3
"""
Launch the host-side comms node for either the PEZ robot or a BlueROV.
Usage examples
--------------

# PEZ  (default)
ros2 launch pez_joy host_launch.py

# BlueROV
ros2 launch pez_joy host_launch.py robot:=bluerov

# Override the YAML that is passed to --config-file
ros2 launch pez_joy host_launch.py robot:=pez \
                       config_file:=/tmp/custom_host.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


# ─────────────────────────────────────────────────────────────────────────────
# Runtime setup (needs the LaunchContext to read the chosen robot)
# ─────────────────────────────────────────────────────────────────────────────
def _launch_setup(context, *args, **kwargs):
    robot = context.launch_configurations.get("robot", "pez")  # 'pez' | 'bluerov'

    # 1) Derive namespace, node-name and default YAML from the robot argument
    ns = robot                                       # → 'pez'  or 'bluerov'
    node_name = f"comms_host_{robot}"                # → 'comms_host_pez' …
    cfg_basename = "host_comms.yaml" \
        if robot == "pez" else "bluerov_comms.yaml"

    default_cfg = PathJoinSubstitution([
        FindPackageShare("pez_comms"),
        "config", cfg_basename
    ])

    # 2) (Re)declare config_file so the user can still override it
    cfg_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_cfg,
        description=f"YAML file passed to --config-file (defaults to {cfg_basename})"
    )

    push_ns = PushRosNamespace(ns)

    node = Node(
        package="pez_comms",
        executable="comms",
        name=node_name,
        output="screen",
        arguments=["--config-file", LaunchConfiguration("config_file")],
    )

    return [cfg_arg, push_ns, node]


# ─────────────────────────────────────────────────────────────────────────────
# generate_launch_description()
# ─────────────────────────────────────────────────────────────────────────────
def generate_launch_description() -> LaunchDescription:
    # The robot selector (pez | bluerov)
    robot_arg = DeclareLaunchArgument(
        "robot",
        default_value=TextSubstitution(text="pez"),
        description='Which robot variant to launch for: "pez" (default) or "bluerov"'
    )

    return LaunchDescription([
        robot_arg,
        OpaqueFunction(function=_launch_setup),
    ])
