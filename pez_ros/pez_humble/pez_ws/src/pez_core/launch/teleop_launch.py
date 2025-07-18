# File: launch/teleop_launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    RegisterEventHandler,
    EmitEvent,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # 3) Declare launch arguments
    test_flag = LaunchConfiguration('test_flag')
    declare_test_flag = DeclareLaunchArgument(
        'test_flag',
        default_value='false',
        description='If true, launch test mode instead of sensors & camera'
    )
    comms_flag = LaunchConfiguration('comms_flag')
    declare_comms_flag = DeclareLaunchArgument(
        'comms_flag',
        default_value='false',
        description='If true, launch accoustic modem mode'
    )

    # 3a) Sensor‐node-specific launch arguments:
    pub_frequency = LaunchConfiguration('publish_frequency')
    declare_pub_frequency = DeclareLaunchArgument(
        'publish_frequency',
        default_value='1.0',
        description='Publish frequency (Hz) for sensors_node'
    )

    pub_tsys01 = LaunchConfiguration('publish_tsys01_temperature')
    declare_pub_tsys01 = DeclareLaunchArgument(
        'publish_tsys01_temperature',
        default_value='true',
        description='Publish TSYS01 temperature? (true/false)'
    )

    pub_ms_temp = LaunchConfiguration('publish_ms5837_temperature')
    declare_pub_ms_temp = DeclareLaunchArgument(
        'publish_ms5837_temperature',
        default_value='true',
        description='Publish MS5837 temperature? (true/false)'
    )

    pub_ms_pres = LaunchConfiguration('publish_ms5837_pressure')
    declare_pub_ms_pres = DeclareLaunchArgument(
        'publish_ms5837_pressure',
        default_value='true',
        description='Publish MS5837 pressure? (true/false)'
    )

    pub_ms_depth = LaunchConfiguration('publish_ms5837_depth')
    declare_pub_ms_depth = DeclareLaunchArgument(
        'publish_ms5837_depth',
        default_value='true',
        description='Publish MS5837 depth? (true/false)'
    )

    fluid_density = LaunchConfiguration('fluid_density')
    declare_fluid_density = DeclareLaunchArgument(
        'fluid_density',
        default_value='997.0',
        description='Fluid density (kg/m³) for MS5837 depth calculation'
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
            {'test_flag': test_flag}
        ],
        condition=UnlessCondition(test_flag),
    )

    # 4b) fish_teleop in test mode (different node name)
    fish_teleop_node_test = Node(
        package='pez_core',
        executable='fish_teleop',
        name='fish_teleop_node_test',
        namespace='pez',
        output='screen',
        parameters=[
            axis_params,
            {'test_flag': test_flag}
        ],
        condition=IfCondition(test_flag),
    )

    pkg_comms   = get_package_share_directory('pez_comms')
    fish_launch = os.path.join(pkg_comms, 'launch', 'fish_launch.py')
    fish_cfg    = os.path.join(pkg_comms, 'config', 'fish_comms.yaml')
    fish_comms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fish_launch),
        launch_arguments={
            'config_file': fish_cfg,
            'namespace':   'pez',
        }.items(),
        condition=IfCondition(comms_flag)
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
            'video_device'    : '/dev/video0'
        }],
        condition=UnlessCondition(test_flag),
    )

    # 6) sensors_node (only if test_flag is false)
    sensors_node = Node(
        package='pez_core',
        executable='fish_sense',
        name='fish_sense_node',
        namespace='pez',
        output='screen',
        parameters=[
            # Pass down the sensor‐node parameters:
            {'publish_frequency': pub_frequency},
            {'publish_tsys01_temperature': pub_tsys01},
            {'publish_ms5837_temperature': pub_ms_temp},
            {'publish_ms5837_pressure': pub_ms_pres},
            {'publish_ms5837_depth': pub_ms_depth},
            {'fluid_density': fluid_density},
        ],
        condition=UnlessCondition(test_flag),
    )

    # 7) Shutdown handlers: if either teleop variant exits, shutdown all
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
        # Environment
        SetEnvironmentVariable('ROS_DOMAIN_ID', '21'),

        # Launch arguments
        declare_test_flag,
        declare_comms_flag,
        declare_pub_frequency,
        declare_pub_tsys01,
        declare_pub_ms_temp,
        declare_pub_ms_pres,
        declare_pub_ms_depth,
        declare_fluid_density,

        # Nodes
        fish_teleop_node,
        fish_teleop_node_test,
        fish_comms,
        usb_cam_node,
        sensors_node,

        # Shutdown on teleop exit
        on_teleop_exit,
        on_teleop_test_exit,
    ])
