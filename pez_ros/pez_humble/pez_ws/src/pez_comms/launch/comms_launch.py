# pez_comms/launch/comms_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    side_arg = DeclareLaunchArgument(
        'side',
        default_value='host',
        description='Which comms node to launch: "host" or "fish"'
    )
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyModem0',
        description='Serial port (or PTY) to open'
    )

    side   = LaunchConfiguration('side')
    port   = LaunchConfiguration('port')

    return LaunchDescription([
        side_arg,
        port_arg,

        # Host-side, only if side=="host"
        Node(
            package='pez_comms',
            executable='host_side',
            name='host_comms',
            output='screen',
            parameters=[{'port': port}],
            condition=IfCondition(PythonExpression([side, " == 'host'"]))
        ),

        # Fish-side, only if side=="fish"
        Node(
            package='pez_comms',
            executable='fish_side',
            name='fish_comms_fish',
            output='screen',
            parameters=[{'port': port}],
            condition=IfCondition(PythonExpression([side, " == 'fish'"]))
        ),
    ])
