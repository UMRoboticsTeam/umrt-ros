#!/usr/bin/env bash

xhost +local:docker

docker run \
	-it \
	--net \
	host \
	-e DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	--volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
	--volume="$(pwd):/workspace" \
	--volume="/tmp/.X11-unix/:/tmp/.X11-unix" \
	--device /dev/dri \
	--rm \
	--name $2 \
	$1

