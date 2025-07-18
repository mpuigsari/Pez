#!/usr/bin/env bash
set -e

# ---------------------------------------------------------------------------
#  Make all newly created files world-writable/readable
umask 000

# ---------------------------------------------------------------------------
#  ROS 2 environment
source /opt/ros/humble/setup.bash
source /pez_ws/install/setup.bash

# ---------------------------------------------------------------------------
#  Decide what to run based on the first CLI argument
MODE="${1:-dev}"        # default to “dev” if no arg given
shift                   # remove the selector, pass the rest through

case "$MODE" in
  dev)
    # Just drop the user into an interactive shell
    exec bash "$@"
    ;;

  cable)
    # Launch as normal (comms_flag defaults to false inside the launch file)
    exec ros2 launch pez_joy pez_launch.py "$@"
    ;;

  pez_comms)
    # Launch with comms_flag set to true
    exec ros2 launch pez_joy pez_launch.py comms_flag:=true robot:=pez "$@"
    ;;

  blue_comms)
    # Launch with comms_flag set to true
    exec ros2 launch pez_joy pez_launch.py comms_flag:=true robot:=bluerov "$@"
    ;;

  *)
    echo "Usage: $0 {dev|cable|pez_comms|blue_comms} [additional ROS 2 args]" >&2
    exit 1
    ;;
esac
