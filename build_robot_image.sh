#!/usr/bin/env bash

docker build \
    -t umrt-ros-robot \
    --build-arg USERNAME=${USER} \
    --target robot_image \
    -f Dockerfile \
    .
