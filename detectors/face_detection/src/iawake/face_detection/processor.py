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
from iawake.core.types import DetectedBox


class FaceDetectionProcessor(Processor):
    return_type = DetectedBox

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

        detections = []
        for (x, y, w, h) in faces:
            detections.append(DetectedBox({
                'x': x,
                'y': y,
                'w': w,
                'h': h,
                'confidence': 1,
            }))
        return detections
