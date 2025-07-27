FROM ros:humble AS robot_image
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Environment Variables
ENV WS=/workspace

# See https://stackoverflow.com/a/76260513. on QEMU aarch64 builds some install script
# ran by libc-bin segfaults. -njreichert
RUN rm /var/lib/dpkg/info/libc-bin.*
RUN apt-get clean
RUN apt-get update

# Dependencies
# Whenever adding deps add a comment before to note what they are for. -njreichert
RUN apt install -y \
# General ROS deps - control, teleop, xacro, etc... -njreichert
	ros-humble-demo-nodes-cpp \
	ros-humble-demo-nodes-py \
	ros-humble-xacro \
	ros-humble-teleop-twist-keyboard \
	ros-humble-ros2-control \
	ros-humble-ros2-controllers \
# Humble - Camera Stuff
	ros-humble-vision-msgs \
	ros-humble-depthai \
	ros-humble-camera-calibration \
	ros-humble-image-transport-plugins \
	ros-humble-image-pipeline \
	ros-humble-diagnostic-updater \
	ros-humble-depth-image-proc \
	ros-humble-foxglove-msgs \
	ros-humble-rviz-imu-plugin \
# Humble - PWM Driver dependencies. 
# See https://github.com/barulicm/PiPCA9685/blob/master/apt_dependencies.txt -njreichert
	pybind11-dev \
	python3-distutils \
	python3-dev \
	libi2c-dev \
	i2c-tools \
# General utilities - for on-device debugging and development. -njreichert
	vim \
	htop \
	xterm \
	python3 \
	python3-pip \
	build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-vcstool \
    wget \
	libusb-1.0-0-dev \
	libcairo2-dev \
	libgirepository1.0-dev 

RUN apt install -y  ros-humble-ros2-socketcan


# ROS2 Dependencies
# TODO: What does this do on the 
# RUN apt install -y --no-install-recommends \
#     && rm -rf /var/lib/apt/lists/*

# Setup Python dependencies
RUN apt-get update && apt-get install -y \
    ninja-build \
    pkg-config \
    libglib2.0-dev \
    libcairo2-dev \
    libgirepository1.0-dev
RUN pip install meson

# Used for depthai stuff. -njreichert
COPY requirements.txt /opt/app/requirements.txt

# Used for depthai stuff. -njreichert
COPY requirements.txt /opt/app/requirements.txt


# Setup
RUN pip install -r /opt/app/requirements.txt

WORKDIR /workspace

ENV SHELL=/bin/bash

# USER $USERNAME
CMD ["/bin/bash"]

FROM robot_image AS dev_image

RUN apt update
RUN apt install -y \
	# Visualization and simulation tools. -njreichert
	ros-humble-rviz2 \
	ros-humble-joint-state-publisher-gui \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros2-control \
	# Tools for working with / connecting to joysticks & gamepads. -njreichert
	joystick \
	jstest-gtk \
	evtest \
	# ROS2 Joystick utilities. -njreichert
	ros-humble-joy \
	ros-humble-joy-teleop \
	ros-humble-teleop-twist-joy \
        # Foxglove for viewing ROS topics
	ros-humble-foxglove-bridge

# CAN Stuff

RUN apt update &&\
    apt install software-properties-common -y &&\
    add-apt-repository ppa:lely/ppa -y &&\
    apt update &&\
    apt install net-tools iproute2 can-utils kmod liblely-coapp-dev liblely-co-tools python3-dcf-tools -y

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN usermod -aG messagebus $USERNAME 

ENV SHELL=/bin/bash

USER $USERNAME
CMD ["/bin/bash"]

FROM robot_image AS build_artifact

ADD . /workspace

WORKDIR /workspace
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

# Enable faster builds with build.sh. -njreichert
ENV COLCON_BUILD_EXECUTOR=parallel 

# TODO: Required? -njreichert
RUN source /opt/ros/humble/setup.bash

RUN rosdep update
RUN apt update

RUN rosdep install --from-paths /workspace/src --ignore-src --rosdistro=humble -y

RUN rm -rf /workspace

# TODO: Look into what exactly is failing on builds of depthai-examples
# only on Dockerfiles. -njreichert
# RUN source /opt/ros/humble/setup.bash && ./build.sh

CMD ["/bin/bash"]

