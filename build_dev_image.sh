#!/usr/bin/env bash

docker build \
    -t umrt-ros-dev \
    --build-arg USERNAME=${USER} \
    -f Dockerfile \
    .
