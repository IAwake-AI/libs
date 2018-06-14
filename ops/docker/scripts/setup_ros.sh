#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
mkdir -p catkin_workspace/src
cd catkin_workspace/src
catkin_init_workspace
cd ../
setup_ros_for_service \
    --external_usb_cam \
    iawake.animator.service.AnimatorService \
    .
catkin_make
