#!/usr/bin/env bash

echo "Building image..."
docker build \
    -t umrt-ros-robot \
    --build-arg USERNAME=${USER} \
    --target build_artifact \
    --platform linux/arm64 \
    -f Dockerfile \
    . \
    "$@"

echo -ne "Saving image...\t"
docker image save umrt-ros-robot -o umrt-ros-robot.tar
echo "Done!"

