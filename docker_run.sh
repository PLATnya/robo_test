#!/usr/bin/env bash

CONTAINER_NAME="ros_gz_gui"
IMAGE_NAME="exentr0/ros2-test:jazzy"

if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    docker start -i ${CONTAINER_NAME}
else
    docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd)/scripts:/root/ros2_ws/src/my_diff_robot/scripts/safety --gpus all --name ${CONTAINER_NAME} ${IMAGE_NAME} 2>/dev/null
fi

