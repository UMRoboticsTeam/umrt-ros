#!/usr/bin/env bash

#docker build \
#    -t umrt-ros-dev \
#    --build-arg USERNAME=${USER} \
#    -f Dockerfile \
#    --target dev_image \
#    .

docker buildx build \
	--platform linux/amd64 \
	-t umrt-ros-dev \
	--build-arg USERNAME=${USER} \
	-f Dockerfile \
	--target dev_image \
	.
