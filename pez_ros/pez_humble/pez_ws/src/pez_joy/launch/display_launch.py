#!/usr/bin/env python3
"""Optional GUI tools: RQT and PlotJuggler.

If the launch is running for a PEZ robot *and* the comms bridge is enabled
(`robot:=pez comms_flag:=true`) we switch to ROS_DOMAIN_ID 22 for the GUI tools.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from ament_index_python.packages import get_package_share_directory

# ──────────────────────────── paths ──────────────────────────────
pkg_core = get_package_share_directory("pez_joy")
perspective = os.path.join(pkg_core, "config", "pez_rqt.perspective")
layout      = os.path.join(pkg_core, "config", "pez_plot.xml")

# ──────────────────────────── arguments ──────────────────────────

display_flag = LaunchConfiguration("display_flag", default="true")
fish_robot        = LaunchConfiguration("fish_robot", default="true",)
comms_flag   = LaunchConfiguration("comms_flag", default="false")

"""# ─────────────────── conditional ROS_DOMAIN_ID=22 ───────────────
pez_comms_cond = PythonExpression(
    ['"', fish_robot, '" == "true" and "', comms_flag, '" == "true" and "', display_flag, '" == "true"']
)

set_domain_22 = SetEnvironmentVariable(
    name="ROS_DOMAIN_ID",
    value="22",
    condition=IfCondition(pez_comms_cond),
rqt_2 = ExecuteProcess(
    cmd=["rqt", "--perspective-file", perspective],
    output="screen",
    condition=IfCondition(pez_comms_cond),
)
)"""

# ────────────────────────── GUI processes ───────────────────────
rqt = ExecuteProcess(
    cmd=["rqt", "--perspective-file", perspective],
    output="screen",
    condition=IfCondition(display_flag),
)

pj  = ExecuteProcess(
    cmd=["ros2", "run", "plotjuggler", "plotjuggler", "--layout", layout],
    output="screen",
    condition=IfCondition(display_flag),
)

# ────────────────────────── launch description ──────────────────
def generate_launch_description():
    return LaunchDescription(
        [
            rqt,
            pj,
        ]
    )
