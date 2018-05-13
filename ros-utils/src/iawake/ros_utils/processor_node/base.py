import rospy
from std_msgs.msg import (
    Int32,
    String,
)

from iawake.core.processor import NoDataAvailable
from iawake.core.utils import load_module_member


def _get_ros_msg_from_return_type(return_type):
    return {
        int: Int32,
        str: String,
    }[return_type]


class ProcessorNode(object):
    subscribing_msg = String

    def __init__(
            self,
            node_name,
            subscribing_topic,
            publishing_topic,
            processor_path,
    ):
        processor_cls = load_module_member(processor_path)
        self._processor = processor_cls()
        ros_msg = _get_ros_msg_from_return_type(self._processor.return_type)
        self._publisher = rospy.Publisher(
            publishing_topic,
            ros_msg,
            queue_size=10,
        )
        rospy.init_node(node_name)
        rospy.Subscriber(
            subscribing_topic,
            self.subscribing_msg,
            self.process,
            queue_size=1,
        )

    def process(self, data):
        try:
            output = self._processor(data)
        except NoDataAvailable:
            return

        self._publisher.publish(output)

    @staticmethod
    def run():
        rospy.spin()
