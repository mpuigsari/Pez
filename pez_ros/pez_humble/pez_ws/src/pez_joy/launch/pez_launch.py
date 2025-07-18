import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ─────────────────────────────── Flags ────────────────────────────────
    display_flag = LaunchConfiguration("display_flag")
    fish_robot = LaunchConfiguration("fish_robot")
    comms_flag = LaunchConfiguration("comms_flag")
    robot = LaunchConfiguration("robot")

    declare_display = DeclareLaunchArgument(
        "display_flag",
        default_value="true",
        description="If true, launch PlotJuggler & RQT",
    )
    declare_fish = DeclareLaunchArgument(
        "fish_robot",
        default_value="true",
        description="If false, run host-side teleop instead of on-robot controller",
    )
    declare_comms = DeclareLaunchArgument(
        "comms_flag",
        default_value="false",
        description="If true, launch the pez_comms bridge",
    )
    declare_robot = DeclareLaunchArgument(
        "robot",
        default_value="pez",
        description="Robot variant to use with the comms bridge (pez or bluerov)",
    )

    set_domain = SetEnvironmentVariable("ROS_DOMAIN_ID", "21")

    # ────────────────────────────── Includes ──────────────────────────────
    pkg_joy = get_package_share_directory("pez_joy")

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_joy, "launch", "joy_launch.py")
        ),
        launch_arguments={
            "fish_robot": fish_robot,
            "comms_flag": comms_flag,
            "robot": robot,
        }.items(),
    )

    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_joy, "launch", "display_launch.py")
        ),
        launch_arguments={"display_flag": display_flag}.items(),
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_joy, "launch", "comms_launch.py")
        ),
        launch_arguments={
            "fish_robot": fish_robot,
            "comms_flag": comms_flag,
            "robot": robot,
        }.items(),
    )

    pkg_core = get_package_share_directory("pez_core")
    teleop_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_core, "launch", "teleop_launch.py")
        ),
        launch_arguments={"test_flag": "true"}.items(),
        condition=UnlessCondition(fish_robot),
    )

    return LaunchDescription(
        [
            set_domain,
            declare_display,
            declare_fish,
            declare_comms,
            declare_robot,
            teleop,
            teleop_include,
            bridge,
            display,
        ]
    )
