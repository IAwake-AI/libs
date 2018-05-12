## Description
This package serves as an example of how to write a service to process a data stream
and test in both a ROS and non-ROS environment. You can easily replicate this setup
by copying this library and setting up your own
[processor](https://github.com/Automa-Cognoscenti/libs/blob/master/emotion/src/iawake/emotion/processor.py)
and [service](https://github.com/Automa-Cognoscenti/libs/blob/master/emotion/src/iawake/emotion/service.py).

## Installation
```bash
$ pip install --process-dependency-links -e .
```

## Non-ROS usage

```bash
$ python -m iawake.emotion.service
```

## ROS usage

1) Install ros-utils
```bash
$ cd libs/ros-utils && pip install --process-dependency-links -e .
```

2) Install ROS

Follow installation instructions [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).

3) Create catkin workspace

```bash
$ mkdir ~/catkin_ws
$ cd ~/catkin_ws
$ mkdir src
$ cd src
$ catkin_init_workspace
$ cd ..
$ setup_ros_for_service iawake.emotion.service.EmotionService .
$ catkin_make
```

4) Launch nodes
```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch src/launch/generated.launch
```
