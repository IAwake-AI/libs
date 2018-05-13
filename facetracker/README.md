## Description
Tracks a face and sends a message to the humanoid robot to turn its head to (very roughly) track that face.

## Installation
```bash
mkdir SOFTWARE
cd SOFTWARE
sudo apt install python python-pip
sudo apt install git 
git clone https://github.com/Automa-Cognoscenti/libs.git 
cd libs/facetracker
$ pip install --process-dependency-links -e .
```

## Usage

1. Launch gazebo (follow the setup instructions in ROS-master)

```bash
roslaunch humanoid gazebo.launch
```

2. In another terminal, launch the facetracker service

```bash
$ python -m iawake.facetracker.service
```

## If it won't release your camera (may be an iSight only issue)

Kill it:

```bash
ps aux|grep facetracker|awk '{print $2}'|xargs kill -9
```
