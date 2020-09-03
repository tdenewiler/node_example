#!/bin/bash

ROS_DISTRO=noetic
PYTHON_VERSION=3  # 3 for noetic and higher, empty if melodic or lower

docker build -t ros_node_example:$ROS_DISTRO --build-arg ROS_DISTRO=$ROS_DISTRO . --build-arg PYTHON_VERSION=$PYTHON_VERSION
