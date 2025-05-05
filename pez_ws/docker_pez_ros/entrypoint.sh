#!/bin/bash
set -e

# 1) source ROS
source /opt/ros/noetic/setup.bash

# 2) source your workspace
if [ -f /root/pez_ws/devel/setup.bash ]; then
  source /root/pez_ws/devel/setup.bash
fi

# 3) drop into whatever command was passed (default: bash)
exec "$@"
