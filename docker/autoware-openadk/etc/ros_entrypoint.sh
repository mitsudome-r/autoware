#!/bin/bash
# Trigger an error if non-zero exit code is encountered
set -e

source /opt/ros/humble/setup.bash
source /autoware/install/setup.bash
exec "$@"