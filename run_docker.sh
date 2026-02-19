#!/bin/bash
xhost +local:docker
docker run -it --rm \
    --net=host \
    --privileged \
    --ipc=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/home/user/robot_ws" \
    --device /dev/video0:/dev/video0 \
    --name my_robot_container \
    my_robot_image
