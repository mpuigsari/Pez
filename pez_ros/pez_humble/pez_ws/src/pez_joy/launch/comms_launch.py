# launch/bridge_launch.py
"""Include the comms bridge for PEZ or BlueROV."""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    comms_flag = LaunchConfiguration("comms_flag")
    fish_robot = LaunchConfiguration("fish_robot")
    robot = LaunchConfiguration("robot")

    pkg_comms = get_package_share_directory("pez_comms")
    test_launch = os.path.join(pkg_comms, "launch", "test_launch.py")
    host_launch = os.path.join(pkg_comms, "launch", "host_launch.py")

    host_condition = PythonExpression(
        ['"', comms_flag, '" == "true" and "', fish_robot, '" == "false"']
    )
    test_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(test_launch),
        condition=IfCondition(host_condition),
    )

    hw_condition = PythonExpression(
        ['"', comms_flag, '" == "true" and "', fish_robot, '" == "true"']
    )
    host_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(host_launch),
        launch_arguments={"robot": robot}.items(),
        condition=IfCondition(hw_condition),
    )

    return LaunchDescription(
        [
            test_bridge,
            host_bridge,
        ]
    )
