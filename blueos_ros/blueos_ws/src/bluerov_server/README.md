# bluerov_server

Utility package originally developed by the CIRTESU team.  It exposes
ROS topics and services to control a BlueROV2 via MAVLink.  The main
script `server.py` subscribes to `/cmd_vel` and sends RC override
commands to the vehicle.

This package is included unchanged for convenience and used together
with the Noetic `pez_comms` node when teleoperating a BlueROV2 over the
acoustic link.
