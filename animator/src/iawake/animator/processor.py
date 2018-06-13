import rospy
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)

from iawake.core.processor import Processor
from iawake.core.types.animations import HeadTilt


class AnimatorProcessor(Processor):
    return_type = JointTrajectory
    publishing_topic = '/humanoid/humanoid_joint_controller/command'

    @staticmethod
    def _process_head_tilt(head_tilt):
        trajectory = JointTrajectory()

        trajectory.header.stamp = rospy.Time.now()
        trajectory = {
            'head/twist': head_tilt.angle,
        }
        trajectory.joint_names = trajectory.keys()

        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(5)
        point.positions = trajectory.values()
        trajectory.points = [point]
        return trajectory

    def process(self, data):
        fn = {
            HeadTilt: self._process_head_tilt,
        }[type(data)]
        return fn(data)
