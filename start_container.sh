#!/usr/bin/env bash
xhost +local:docker

docker run \
	-it \
	-e DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	-e ROS_DOMAIN_ID=8 \
	--privileged \
	--volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
	--volume="$(pwd):/workspace" \
	--volume="/tmp/.X11-unix/:/tmp/.X11-unix" \
	--volume="/dev:/dev" \
	--device-cgroup-rule="c 189:* rw" \
	--platform linux/amd64 \
	--device /dev/input \
	--network=host \
	--cap-add=NET_ADMIN \
	--rm \
	--pid=host \
	--name $2 \
	$1
