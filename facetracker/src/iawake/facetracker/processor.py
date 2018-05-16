
from __future__ import division

import os

import cv2
from keras.models import model_from_json
from keras.preprocessing import image
import numpy as np
import rospy
import tensorflow as tf

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64

from iawake.core.processor import (
    NoDataAvailable,
    Processor,
)

class FaceProcessor(Processor):
    return_type=int
    def __init__(self):
        models_directory = os.path.dirname(os.path.realpath(__file__)) + '/models'
        cascade_path = os.path.join(
            models_directory,
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
	    return x + w/2

class RobotProcessor(Processor):
   return_type=JointTrajectory

   def process(self,data):
	angle = 1.57 * (1 - data / 360)
	#rospy.init_node('joint_control')

    	#pub = rospy.Publisher('/humanoid/humanoid_joint_controller/command', JointTrajectory, queue_size=10)
    	trajectory = JointTrajectory()

    	trajectory.header.stamp = rospy.Time.now()
    	trajectory.joint_names= ['leg/left/hip/bend/forward', 'leg/left/hip/bend/side', 'leg/left/hip/twist', 'leg/right/hip/bend/forward', 'leg/right/hip/bend/side', 'leg/right/hip/twist', 'leg/left/knee', 'leg/right/knee', 'hand/left/shoulder/bend/forward', 'hand/left/shoulder/bend/side', 'hand/right/shoulder/bend/forward', 'hand/right/shoulder/bend/side', 'head/bend/forward', 'head/bend/side', 'head/twist']
    
    	point = JointTrajectoryPoint()
    	point.time_from_start = rospy.Duration(5)
    	point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, angle]

    	trajectory.points = [point]

    	#pub.publish(trajectory)
	return trajectory
