#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/../"
source "$WORKSPACE_ROOT/amd64.env"
if [ "$(uname -m)" = "aarch64" ]; then
    source "$WORKSPACE_ROOT/arm64.env"
fi

# Function to print help message
print_help() {
    echo "Usage: run.sh [OPTIONS]"
    echo "Options:"
    echo "  --help          Display this help message"
    echo "  -h              Display this help message"
    echo "  --no-nvidia     Disable NVIDIA GPU support"
    echo "  --devel         Use the latest development version of Autoware"
    echo "  --headless      Run Autoware in headless mode"
    echo "  --workspace     Specify the workspace path (default: current directory)"
    echo "  --map-path      Specify the path to the map files (mandatory)"
    echo "  --launch-cmd    Specify the launch command (default: autoware.launch.xml)"
    echo ""
    echo "Note: The --map-path option is mandatory. Please provide the path to the map files."
}

# Default values
option_no_nvidia=false
option_devel=false
option_headless=false
MAP_PATH="/path/to/map"
LAUNCH_CMD="ros2 launch autoware_launch autoware.launch.xml map_path:=${MAP_PATH} vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit"
WORKSPACE_PATH="${PWD}"
USER_ID=""

# Parse arguments
parse_arguments() {
    while [ "$1" != "" ]; do
        case "$1" in
        --help | -h)
            print_help
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
        esac
        shift
    done
}

# Set image and workspace variables based on options
set_variables() {
    if [ "$option_devel" == "true" ]; then
        IMAGE="ghcr.io/autowarefoundation/autoware-openadk:devel-$rosdistro-latest"
        USER_ID="-e LOCAL_UID=$(id -u) -e LOCAL_GID=$(id -g) -e LOCAL_USER=$(id -un) -e LOCAL_GROUP=$(id -gn)"
        WORKSPACE="-v ${WORKSPACE_PATH}:/workspace"
        LAUNCH_CMD=""
    else
        if [ "$MAP_PATH" == "" ]; then
            print_help
            exit 1
        fi
        MAP="-v ${MAP_PATH}:/${MAP_PATH}"
        IMAGE="ghcr.io/autowarefoundation/autoware-openadk:monorun-$rosdistro-latest"
        WORKSPACE=""
    fi
}

# Set GPU flag based on option
set_gpu_flag() {
    if [ "$option_no_nvidia" = "false" ]; then
        IMAGE=${IMAGE}-cuda
        GPU_FLAG="--gpus all"
    else
        GPU_FLAG=""
    fi
}

# Set X display variables
set_x_display() {
    MOUNT_X=""
    if [ "$option_headless" = "false" ]; then
        MOUNT_X="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix"
        xhost +
    fi
}

# Main script execution
main() {
    parse_arguments "$@"
    set_variables
    set_gpu_flag
    set_x_display

    # Launch Autoware with custom map
    docker run -it --rm --net=host ${GPU_FLAG} ${USER_ID} ${MOUNT_X} \
        ${WORKSPACE} ${MAP} ${IMAGE} \
        ${LAUNCH_CMD}
}

# Execute the main script
main "$@"
