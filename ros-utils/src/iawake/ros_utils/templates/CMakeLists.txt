cmake_minimum_required(VERSION 2.8.3)
project({{ pkg_name }})

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
