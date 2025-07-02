#!/usr/bin/env bash

# Based on:
# http://wiki.ros.org/docker/Tutorials/GUI

IMAGE_NAME=ros2-humble-ntt
CONTAINER_NAME=px4_agent_ws
XSOCK="/tmp/.X11-unix"
MOUNT_PATH=$(realpath "$HOME/px4_agent_ws")

# Allow Docker to access host X11
xhost +local:root > /dev/null

# ====================================
# Check if the container with the same name already exists.
# ====================================
if [ ! "$(docker ps -a | grep $CONTAINER_NAME)" ]; then
    docker run -it \
        --privileged \
        --env="DISPLAY=$DISPLAY" \
        --workdir="/home/ros2_ws/" \
        --volume="/dev:/dev" \
        --volume="${XSOCK}:${XSOCK}:rw" \
        --volume="${HOME}/.ssh:/root/.ssh:ro" \
        --device /dev/dri \
        --volume="/var/run/dbus:/var/run/dbus" \
        --volume="${MOUNT_PATH}/src:/home/ros2_ws/src" \
        --network host \
        --runtime=runc \
        --name "$CONTAINER_NAME" \
        "$IMAGE_NAME" \
        "$@"
else
    echo "Container with the same name already exists, executing it:"
    docker start "$CONTAINER_NAME" > /dev/null
    docker exec -it "$CONTAINER_NAME" bash
fi

# Revoke X11 access
xhost -local:root > /dev/null
