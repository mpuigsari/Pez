#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

# overlay your workspace, if it exists
if [ -f /pez_ws/install/setup.bash ]; then
  source /pez_ws/install/setup.bash
fi

exec "$@"
