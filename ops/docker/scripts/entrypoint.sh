#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source catkin_workspace/devel/setup.bash
roslaunch catkin_workspace/src/launch/generated.launch
