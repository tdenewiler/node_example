#!/bin/bash

# After container starts it can be helpful to run:
# rosdep install --from-paths src --ignore-src -r -y --os=ubuntu:focal --rosdistro noetic
# Might want to do:
# sudo apt install python-is-python3
# sudo -H pip install black

# Change the workspace (WS) variable to match the location where you have the node_example workspace on the host.

ROS_DISTRO=noetic
WS=/home/$USER/ws

docker run --name ros_node_example -v $WS:/home/ros/ws -ti ros_node_example:$ROS_DISTRO /bin/bash
