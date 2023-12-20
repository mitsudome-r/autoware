#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /autoware/install/setup.bash
exec "$@"
