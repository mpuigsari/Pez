# File: launch/pez_with_camera.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('pez_control')

    # 1) Your axis_controller params
    axis_params = os.path.join(pkg, 'config', 'axis_params.yaml')

    # 2) Path to the camera_info YAML
    calib_yaml = os.path.join(pkg, 'config', 'default_cam.yaml')
    # Make sure the URL is absolute and prefixed with file://
    camera_info_url = f'file://{calib_yaml}'

    # 3) Build the Node definitions
    axis_node = Node(
        package='pez_control',
        executable='axis_controller',
        name='pez_controller',
        namespace='pez',
        output='screen',
        parameters=[ axis_params ],
    )

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='pez',
        output='screen',
        parameters=[{
            'pixel_format'    : 'mjpeg2rgb',   # hardware MJPEG decode → rgb8
            'image_width'     : 640,
            'image_height'    : 480,
            'framerate'       : 30.0,
            'camera_name'     : 'default_cam',
            'camera_info_url' : camera_info_url,
        }],
    )

    return LaunchDescription([
        axis_node,
        usb_cam_node,
    ])
