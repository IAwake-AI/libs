import rospy

from iawake.ros_utils.node.base import Node


class FeedNode(Node):
    def __init__(self, feed_cls, *args, **kwargs):
        super(FeedNode, self).__init__(*args, **kwargs)
        self._rate = rospy.Rate(10)
        self._feed = feed_cls()

    def run(self):
        while not rospy.is_shutdown():
            data = self._feed.get_data()
            self._publish(data)
            self._rate.sleep()
