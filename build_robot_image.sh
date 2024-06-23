#!/usr/bin/env bash

docker build \
    -t umrt-ros \
    --build-arg USERNAME=${USER} \
    --target robot_image \
    -f $1 \
    .
