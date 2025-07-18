# `pez_joy` Package

`pez_joy` contains the joystick driven tele‑operation stack for the PEZ project. It publishes velocity commands, camera angles and service triggers while also providing helper launch files to start the full teleop pipeline.

## Features

- ROS 2 node `pez_joy` that converts joystick messages into `cmd_vel`, camera and service calls.
- Supports snapshot logging of sensor values whenever the electromagnet is deactivated.
- Launch files to run on the robot or on a host laptop with optional acoustic bridge and display tools.
- YAML configuration of joystick axes, buttons and scaling factors.
- Experimental `command_player` script for time scripted missions.

## Launch Files

- `joy_launch.py` – base teleoperation; parameters `robot`, `comms_flag`, `fish_robot` and `joy_dev`.
- `display_launch.py` – RQT and PlotJuggler windows when `display_flag` is true.
- `comms_launch.py` – helper to include the `pez_comms` bridge.
- `pez_launch.py` – convenience wrapper that combines all of the above.
- `experiment_launch.py` – runs the `command_player` using a YAML file.

Typical usage on the robot side:

```bash
ros2 launch pez_joy pez_launch.py comms_flag:=true robot:=pez
```

For host teleoperation:

```bash
ros2 launch pez_joy pez_launch.py fish_robot:=false display_flag:=true
```

## Snapshots Directory

Magnet off events trigger a call to `pez_interfaces/srv/SnapSensors` and store CSV files under `share/pez_joy/snapshots/`.
