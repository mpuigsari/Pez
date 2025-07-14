#!/usr/bin/env bash
set -e  # exit on first error

# ---------------------------------------------------------------
#  Source the ROS 2 environments
source /opt/ros/humble/setup.bash
source /pez_ws/install/setup.bash

# ---------------------------------------------------------------
#  Hand over to an interactive login shell.
#  "$@" lets you add commands after `docker compose run …`
exec bash -l "$@"
