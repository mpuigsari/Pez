# File: launch/pez_with_camera.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
    RegisterEventHandler,
    EmitEvent,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    pkg = get_package_share_directory('pez_core')

    # 1) Your axis_controller params
    axis_params = os.path.join(pkg, 'config', 'axis_params.yaml')

    # 2) Path to the camera_info YAML
    calib_yaml = os.path.join(pkg, 'config', 'default_cam.yaml')
    camera_info_url = f'file://{calib_yaml}'

    # 3) Declare test_flag argument
    test_flag = LaunchConfiguration('test_flag')
    declare_test_flag = DeclareLaunchArgument(
        'test_flag',
        default_value='false',
        description='If true, launch PlotJuggler instead of the camera node'
    )

    # 4) PezController node (always)
    axis_node = Node(
        package='pez_core',
        executable='fish_teleop',
        name='fish_teleop_node',
        namespace='pez',
        output='screen',
        parameters=[
            axis_params,
            { 'test_flag': test_flag }  # pass through to your node
        ],
    )

    # 5) USB camera node (only if test_flag is false)
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='pez',
        output='screen',
        parameters=[{
            'pixel_format'    : 'mjpeg2rgb',
            'image_width'     : 640,
            'image_height'    : 480,
            'framerate'       : 30.0,
            'camera_name'     : 'default_cam',
            'camera_info_url' : camera_info_url,
        }],
        condition=UnlessCondition(test_flag),
    )

    # 6) PlotJuggler if testing
    plotjuggler = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'plotjuggler', 'plotjuggler',
        ],
        output='screen',
        condition=IfCondition(test_flag),
    )

    # 7) Shutdown handlers: if either exits, tear down the whole launch
    on_axis_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=axis_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )
    on_pj_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=plotjuggler,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '21'),
        declare_test_flag,
        axis_node,
        usb_cam_node,
        plotjuggler,
        on_axis_exit,
        on_pj_exit,
    ])
