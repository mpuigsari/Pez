from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    ip = LaunchConfiguration("server_ip").perform(context)

    actions = []

    if ip:
        # If server_ip is specified, bind to it
        actions.append(
            LogInfo(msg=f"[iperf3 server] Binding to specified IP: {ip}")
        )
        actions.append(
            ExecuteProcess(
                cmd=["stdbuf", "-oL", "iperf3", "-s", "-B", ip],
                name="iperf3_server",
                output="screen"
            )
        )
    else:
        # Default: bind to all interfaces
        actions.append(
            LogInfo(msg="[iperf3 server] No server_ip specified — binding to all interfaces (0.0.0.0)")
        )
        actions.append(
            ExecuteProcess(
                cmd=["stdbuf", "-oL", "iperf3", "-s"],
                name="iperf3_server",
                output="screen"
            )
        )

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "server_ip",
            default_value="",
            description="Optional IP address to bind the iperf3 server to"
        ),
        OpaqueFunction(function=launch_setup)
    ])
