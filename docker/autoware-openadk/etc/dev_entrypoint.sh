#!/bin/bash

USER_ID=${LOCAL_UID}
GROUP_ID=${LOCAL_GID}

# Extract username from /etc/passwd based on the passed UID
USER_NAME=$(getent passwd "$USER_ID" | cut -d: -f1)

if [ -z "$USER_NAME" ]; then
    echo "User with UID $USER_ID not found in /etc/passwd."
    exit 1
fi

echo "Starting with user: $USER_NAME >> UID $USER_ID, GID: $GROUP_ID"
useradd -u $USER_ID -g $GROUP_ID -s /bin/bash -m -d /home/$USER_NAME $USER_NAME

chown -R $USER_NAME:$USER_NAME /home/$USER_NAME

export HOME=/home/$USER_NAME

exec /usr/sbin/gosu $USER_NAME "$@"

source /opt/ros/humble/setup.bash