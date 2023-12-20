#!/usr/bin/env bash

set -e

if [ "$2" == "" ]; then
    echo "Usage: run.sh --map-path path_to_map_files"
    exit 1
fi

# Get the map path
while [ "$1" != "" ]; do
    case "$1" in
    --map-path)
        MAP_PATH="$2"
        shift
        ;;
    *)
        args+=("$1")
        ;;
    esac
    shift
done

# Launch Autoware with custom map
echo "Launching Autoware with the map >> "${MAP_PATH}
docker run -it --rm --net=host --gpus all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix \
    -v ${MAP_PATH}:/autoware_map ghcr.io/autowarefoundation/autoware-openadk:monolithic-humble-latest-cuda \
    ros2 launch autoware_launch autoware.launch.xml map_path:=/autoware_map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
