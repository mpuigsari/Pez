import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('pez_control')
    param_file = os.path.join(pkg_share, 'config', 'axis_params.yaml')

    return LaunchDescription([
        Node(
            package='pez_control',        # your ROS2 package name
            executable='axis_controller',  # name of your console_script entrypoint
            name='pez_controller',       # ROS node name
            output='screen',
            namespace='pez',
            parameters=[param_file],      # load all params from the YAML
        )
    ])
