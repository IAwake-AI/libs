import rospy
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)

from iawake.core.processor import Processor
from iawake.core.types.animations import (
    get_animation_from_wrapper,
    HeadTilt,
)


class AnimatorProcessor(Processor):
    return_type = JointTrajectory
    publishing_topic = '/humanoid/humanoid_joint_controller/command'

    @staticmethod
    def _get_trajectory_info(specified_info):
        joint_names = [
            'leg/left/hip/bend/forward',
            'leg/left/hip/bend/side',
            'leg/left/hip/twist',
            'leg/right/hip/bend/forward',
            'leg/right/hip/bend/side',
            'leg/right/hip/twist',
            'leg/left/knee',
            'leg/right/knee',
            'hand/left/shoulder/bend/forward',
            'hand/left/shoulder/bend/side',
            'hand/right/shoulder/bend/forward',
            'hand/right/shoulder/bend/side',
            'head/bend/forward',
            'head/bend/side',
            'head/twist',
        ]
        info = {name: 0.0 for name in joint_names}
        info.update(**specified_info)
        return info

    @classmethod
    def _process_head_tilt(cls, head_tilt):
        trajectory = JointTrajectory()

        trajectory.header.stamp = rospy.Time.now()
        trajectory_info = cls._get_trajectory_info({
            'head/twist': head_tilt.angle,
        })
        trajectory.joint_names = trajectory_info.keys()

        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(5)
        point.positions = trajectory_info.values()
        trajectory.points = [point]
        return trajectory

    def process(self, data):
        animation = get_animation_from_wrapper(data)
        fn = {
            HeadTilt: self._process_head_tilt,
        }[type(animation)]
        return fn(animation)
