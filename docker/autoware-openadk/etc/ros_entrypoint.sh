#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
source /autoware/install/setup.bash
exec "$@"
