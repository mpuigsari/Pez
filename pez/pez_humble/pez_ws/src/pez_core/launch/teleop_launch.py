# File: launch/pez_with_camera.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
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


    # 4a) fish_teleop in normal mode
    fish_teleop_node = Node(
        package='pez_core',
        executable='fish_teleop',
        name='fish_teleop_node',
        namespace='pez',
        output='screen',
        parameters=[
            axis_params,
            { 'test_flag': test_flag }
        ],
        condition=UnlessCondition(test_flag),
    )

    # 4b) fish_teleop in test mode with a different node name
    fish_teleop_node_test = Node(
        package='pez_core',
        executable='fish_teleop',
        name='fish_teleop_node_test',
        namespace='pez',
        output='screen',
        parameters=[
            axis_params,
            { 'test_flag': test_flag }
        ],
        condition=IfCondition(test_flag),
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


    # 7) Shutdown handlers: if either exits, tear down the whole launch
    on_teleop_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=fish_teleop_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )
    on_teleop_test_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=fish_teleop_node_test,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '21'),
        declare_test_flag,
        fish_teleop_node,
        fish_teleop_node_test,
        usb_cam_node,
        on_teleop_exit,
        on_teleop_test_exit,
    ])
