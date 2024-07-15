#!/usr/bin/env bash
xhost +local:docker

docker run \
	-it \
	-e DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	--privileged \
	--volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
	--volume="$(pwd):/workspace" \
	--volume="/tmp/.X11-unix/:/tmp/.X11-unix" \
	--volume="/dev:/dev" \
	--device-cgroup-rule="c 189:* rw" \
	--device /dev/dri \
	--device /dev/input \
	--rm \
	--network=host \
	--pid=host \
	--name $2 \
	$1
