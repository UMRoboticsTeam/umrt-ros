#!/usr/bin/env bash

docker build \
    -t umrt-ros \
    --build-arg USERNAME=${USER}
    -f $1 \
    .