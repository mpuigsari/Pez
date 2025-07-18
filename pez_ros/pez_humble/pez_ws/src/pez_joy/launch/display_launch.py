# launch/display_launch.py
"""Optional GUI tools: RQT and PlotJuggler."""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    display_flag = LaunchConfiguration("display_flag")
    pkg_core = get_package_share_directory("pez_core")

    perspective = os.path.join(pkg_core, "config", "pez_rqt.perspective")
    layout = os.path.join(pkg_core, "config", "pez_plot.xml")

    rqt = ExecuteProcess(
        cmd=["rqt", "--perspective-file", perspective],
        output="screen",
        condition=IfCondition(display_flag),
    )
    pj = ExecuteProcess(
        cmd=["ros2", "run", "plotjuggler", "plotjuggler", "--layout", layout],
        output="screen",
        condition=IfCondition(display_flag),
    )

    return LaunchDescription(
        [
            rqt,
            pj,
        ]
    )
