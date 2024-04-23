FROM ros:humble

RUN apt update && apt upgrade -y
RUN apt install -y \
	ros-humble-rviz2 \
	ros-humble-demo-nodes-cpp \
	ros-humble-demo-nodes-py \
	ros-humble-xacro \
	ros-humble-joint-state-publisher-gui \
	ros-humble-gazebo-ros-pkgs \
	ros-humble-teleop-twist-keyboard \
	ros-humble-ros2-control \
	ros-humble-ros2-controllers \
	ros-humble-gazebo-ros2-control \
	vim \
	htop 
	
WORKDIR /workspace

CMD ["/bin/bash"]
