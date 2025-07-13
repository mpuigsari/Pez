#!/usr/bin/env bash
set -e

# --- ROS 2 environment -------------------------------------------------------
source /opt/ros/humble/setup.bash
source /pez_ws/install/setup.bash

# --- Decide which value the launch file should get ---------------------------
if [[ "$1" == "comms" ]]; then
  comms_flag=true
  shift          # drop the selector so the launch file never sees it
else
  comms_flag=false
fi

# --- Run the launch file (replace the shell with it) -------------------------
exec ros2 launch pez_core teleop_launch.py \
     comms_flag:=${comms_flag}
     
exec "$@"
