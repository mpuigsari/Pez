#!/usr/bin/env bash
set -e

# 1) Make all newly created files world-writable/readable
umask 000

# 2) Source ROS 2 Humble environment
source /opt/ros/humble/setup.bash
cd /pez_ws

# 3) Source the workspace (if it's already been built)
if [ -f /pez_ws/install/setup.bash ]; then
  source /pez_ws/install/setup.bash
  ros2 launch pez_core
fi

# 5) Exec the given command (defaults to `bash` in your compose)
exec "$@"
