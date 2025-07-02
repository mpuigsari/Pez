# launch/bluerov_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ────────────────────────── Defaults & Launch-args ──────────────────────
    ns      = LaunchConfiguration('namespace',      default='bluerov')
    port    = LaunchConfiguration('port',           default='/dev/ttyUSB1')

    # comms YAML (already shipped in pez_comms/config)
    comms_cfg = PathJoinSubstitution([
        FindPackageShare('pez_comms'), 'config', 'bluerov_comms.yaml'
    ])
    # joystick mapping YAML
    teleop_cfg = PathJoinSubstitution([
        FindPackageShare('pez_comms'), 'config', 'bluerov_joy_config.yaml'
    ])
    joy_dev  = LaunchConfiguration('joy_dev',       default='/dev/input/js0')

    return LaunchDescription([

        # ───────── Allow overrides from the command line ─────────
        DeclareLaunchArgument('namespace',     default_value=ns,
                              description='Namespace for all Bluerov nodes'),
        DeclareLaunchArgument('port',          default_value=port,
                              description='Serial port for the modem'),
        DeclareLaunchArgument('comms_config',  default_value=comms_cfg,
                              description='bluerov_comms.yaml path'),
        DeclareLaunchArgument('teleop_config', default_value=teleop_cfg,
                              description='bluerov_joy_config.yaml path'),
        DeclareLaunchArgument('joy_dev',       default_value=joy_dev,
                              description='Joystick device (/dev/input/jsX)'),

        # ───────── Push the namespace so every node lives under it ───────────
        PushRosNamespace(ns),

        # ───────── Comms node (unchanged) ────────────────────────────────────
        Node(
            package='pez_comms',
            executable='comms',
            name='comms_bluerov',
            output='screen',
            arguments=['--config-file', LaunchConfiguration('comms_config')],
        ),

        # ───────── joy_node  →  /joy ─────────────────────────────────────────
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'dev': LaunchConfiguration('joy_dev')}],
        ),

        # ───────── joy_teleop  (our new bridge) ─────────────────────────────
        # Assumes you installed the script as an entry-point called “joy_teleop”
        Node(
            package='pez_comms',               # adjust if you placed it elsewhere
            executable='joy_teleop',
            name='joy_teleop',
            output='screen',
            arguments=['-c', LaunchConfiguration('teleop_config')],
        ),
    ])
