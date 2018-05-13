from __future__ import division

import os

import cv2
import rospy
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)

from iawake.core.processor import (
    NoDataAvailable,
    Processor,
)


class FaceProcessor(Processor):
    return_type = int

    def __init__(self):
        models_directory = os.path.dirname(os.path.realpath(__file__))
        cascade_path = os.path.join(
            models_directory,
            'models',
            'haarcascade_frontalface_default.xml',
        )
        self.face_cascade = cv2.CascadeClassifier(cascade_path)

    def process(self, data):
        img = data

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

        if len(faces) == 0:
            raise NoDataAvailable

        for (x, y, w, h) in faces:
            return x + w / 2


class RobotProcessor(Processor):
    return_type = JointTrajectory

    def process(self, data):
        angle = 1.57 * (1 - data / 360)
        trajectory = JointTrajectory()

        trajectory.header.stamp = rospy.Time.now()
        trajectory = {
            'leg/left/hip/bend/forward': 0.0,
            'leg/left/hip/bend/side': 0.0,
            'leg/left/hip/twist': 0.0,
            'leg/right/hip/bend/forward': 0.0,
            'leg/right/hip/bend/side': 0.0,
            'leg/right/hip/twist': 0.0,
            'leg/left/knee': 0.0,
            'leg/right/knee': 0.0,
            'hand/left/shoulder/bend/forward': 0.0,
            'hand/left/shoulder/bend/side': 0.0,
            'hand/right/shoulder/bend/forward': 0.0,
            'hand/right/shoulder/bend/side': 0.0,
            'head/bend/forward': 0.0,
            'head/bend/side': 0.0,
            'head/twist': angle,
        }
        trajectory.joint_names = trajectory.keys()

        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(5)
        point.positions = trajectory.values()
        trajectory.points = [point]
        return trajectory
