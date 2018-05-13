cmake_minimum_required(VERSION 2.8.3)
project({{ pkg_name }})

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  message_generation
)

{% if msg_files|length > 0 %}
add_message_files(
  FILES
  {{ msg_files }}
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
{% endif %}

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})
