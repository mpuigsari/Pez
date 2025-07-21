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
MODE="${1:-pez}"        # default to “dev” if no arg given
shift                   # remove the selector, pass the rest through

case "$MODE" in
  dev)
    # Just drop the user into an interactive shell
    exec bash "$@"
    ;;

  pez)
    exec ros2 launch pez_comms host_launch.py  robot:=pez"$@"
    ;;
  
  bluerov)
    exec ros2 launch pez_comms host_launch.py  robot:=bluerov"$@"
    ;;

  *)
    echo "Usage: $0 {dev|pez|bluerov} [additional ROS 2 args]" >&2
    exit 1
    ;;
esac
