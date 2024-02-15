#!/bin/bash

# Get the user ID and group ID of the local user
USER_ID=${LOCAL_UID}
USER_NAME=${LOCAL_USER}
GROUP_ID=${LOCAL_GID}
GROUP_NAME=${LOCAL_GROUP}
echo "Starting with user: $USER_NAME >> UID $USER_ID, GID: $GROUP_ID"

# Create group and user with GID/UID
groupadd -g $GROUP_ID  $GROUP_NAME
useradd -u $USER_ID -g $GROUP_ID -s /bin/bash -m -d /home/$USER_NAME $USER_NAME

# Add sudo privileges to the user
echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Source ROS2
source /opt/ros/humble/setup.bash

# Execute the command as the user
exec /usr/sbin/gosu $USER_NAME "$@"