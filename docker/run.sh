#!/usr/bin/env bash

set -e

DESCRIPTION=""

option_no_nvidia=false
option_devel=false
option_headless=false
LAUNCH_CMD="ros2 launch autoware_launch autoware.launch.xml map_path:=/autoware_map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit"
MAP_PATH=""
WORKSPACE_PATH="${PWD}"
USER_ID=""
args=()

# Parse arguments
while [ "$1" != "" ]; do
    case "$1" in
    --help)
        echo "$DESCRIPTION"
        exit 1
        ;;
    -h)
        echo "$DESCRIPTION"
        exit 1
        ;;
    --no-nvidia)
        option_no_nvidia=true
        ;;
    --devel)
        option_devel=true
        ;;
    --headless)
        option_headless=true
        ;;
    --workspace)
        WORKSPACE_PATH="$2"
        shift
        ;;
    --map-path)
        MAP_PATH="$2"
        shift
        ;;
    --launch-cmd)
        LAUNCH_CMD="$2"
        shift
        ;;
    *)
        args+=("$1")
        ;;
    esac
    shift
done

if [ "$option_devel" == "true" ]; then
    IMAGE="ghcr.io/autowarefoundation/autoware-openadk:latest-devel-humble"
    USER_ID="-e LOCAL_UID=$(id -u) -e LOCAL_GID=$(id -g)"
    WORKSPACE="-v ${WORKSPACE_PATH}:/workspace"
    LAUNCH_CMD=""
else
    if [ "$MAP_PATH" == "" ]; then
        echo "Usage: run.sh --map-path path_to_map_files"
        exit 1
    fi
    MAP="-v ${MAP_PATH}:/autoware_map"
    IMAGE="ghcr.io/autowarefoundation/autoware-openadk:latest-monorun-humble"
    WORKSPACE=""
fi

if [ "$option_no_nvidia" = "false" ]; then
    IMAGE=${IMAGE}-cuda
    GPU_FLAG="--gpus all"
else
    GPU_FLAG=""
fi

if [ "$option_headless" = "false" ]; then
    MOUNT_X="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix"
    xhost +
else
    MOUNT_X=""
fi

# Launch Autoware with custom map
docker run -it --rm --net=host ${GPU_FLAG} ${USER_ID} ${MOUNT_X} \
${WORKSPACE} ${MAP} ${IMAGE} \
${LAUNCH_CMD}
