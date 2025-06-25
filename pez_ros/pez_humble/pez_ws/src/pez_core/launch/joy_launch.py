import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ——————————————————————————————————————————————————
    # 1) Declare flags & environment
    # ——————————————————————————————————————————————————
    display_flag = LaunchConfiguration('display_flag')
    fish_robot   = LaunchConfiguration('fish_robot')
    comms_flag   = LaunchConfiguration('comms_flag')

    declare_display = DeclareLaunchArgument(
        'display_flag', default_value='true',
        description='If true, launch PlotJuggler & RQT'
    )
    declare_fish    = DeclareLaunchArgument(
        'fish_robot', default_value='true',
        description='If false, run host-side teleop instead of on-robot controller'
    )
    declare_comms   = DeclareLaunchArgument(
        'comms_flag', default_value='false',
        description='If true, launch the pez_comms bridge'
    )

    set_domain = SetEnvironmentVariable('ROS_DOMAIN_ID', '21')

    # ——————————————————————————————————————————————————
    # 2) Teleop grouping logic
    # ——————————————————————————————————————————————————
    pkg_core   = get_package_share_directory('pez_core')
    joy_params = os.path.join(pkg_core, 'config', 'joystick_params.yaml')

    # /pez if no comms OR on-robot teleop
    pez_condition = PythonExpression([
        '"', comms_flag, '" == "false" or "', fish_robot, '" == "true"'
    ])
    pez_teleop_group = GroupAction(
        actions=[
            PushRosNamespace('pez'),
            Node(
                package='joy', executable='joy_node', name='joy_node', output='screen',
                parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.05}],
            ),
            Node(
                package='pez_core', executable='fish_joy', name='fish_joy_node', output='screen',
                parameters=[joy_params],
            ),
        ],
        condition=IfCondition(pez_condition)
    )

    # /host if comms AND host-side teleop
    host_condition = PythonExpression([
        '"', comms_flag, '" == "true" and "', fish_robot, '" == "false"'
    ])
    host_teleop_group = GroupAction(
        actions=[
            PushRosNamespace('host'),
            Node(
                package='joy', executable='joy_node', name='joy_node', output='screen',
                parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.05}],
            ),
            Node(
                package='pez_core', executable='fish_joy', name='fish_joy_node', output='screen',
                parameters=[joy_params],
            ),
        ],
        condition=IfCondition(host_condition)
    )

    # ——————————————————————————————————————————————————
    # 3) Host-side teleop include when fish_robot == false
    # ——————————————————————————————————————————————————
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_core, 'launch', 'teleop_launch.py')
        ),
        launch_arguments={'test_flag': 'true'}.items(),
        condition=UnlessCondition(fish_robot)
    )

    # ——————————————————————————————————————————————————
    # 4) Comms bridge logic
    # ——————————————————————————————————————————————————
    pkg_comms   = get_package_share_directory('pez_comms')
    test_launch = os.path.join(pkg_comms, 'launch', 'test_launch.py')
    host_launch = os.path.join(pkg_comms, 'launch', 'host_launch.py')
    host_cfg    = os.path.join(pkg_comms, 'config', 'host_comms.yaml')

    test_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(test_launch),
        condition=IfCondition(host_condition)
    )

    hw_condition = PythonExpression([
        '"', comms_flag, '" == "true" and "', fish_robot, '" == "true"'
    ])
    host_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(host_launch),
        launch_arguments={
            'config_file': host_cfg,
            'namespace':   'pez',
        }.items(),
        condition=IfCondition(hw_condition)
    )

    # ——————————————————————————————————————————————————
    # 5) RQT & PlotJuggler
    # ——————————————————————————————————————————————————
    perspective = os.path.join(pkg_core, 'config', 'pez.perspective')
    layout      = os.path.join(pkg_core, 'config', 'test.xml')

    rqt = ExecuteProcess(
        cmd=['rqt', '--perspective-file', perspective], output='screen',
        condition=IfCondition(display_flag),
    )
    pj  = ExecuteProcess(
        cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler', '--layout', layout], output='screen',
        condition=IfCondition(display_flag),
    )

    # ——————————————————————————————————————————————————
    # 6) Assemble LaunchDescription
    # ——————————————————————————————————————————————————
    return LaunchDescription([
        #set_domain,
        declare_display,
        declare_fish,
        declare_comms,

        pez_teleop_group,
        host_teleop_group,

        teleop_launch,

        test_bridge,
        host_bridge,

        rqt,
        pj,
    ])
