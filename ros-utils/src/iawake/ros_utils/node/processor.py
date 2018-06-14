import rospy

from iawake.core.processor import NoDataAvailable
from iawake.core.types.types import List
from iawake.ros_utils.node.base import Node


class ProcessorNode(Node):
    def __init__(
            self,
            subscribing_topic,
            subscribing_msg,
            subscribing_msg_type,
            processor_cls,
            *args,
            **kwargs
    ):
        super(ProcessorNode, self).__init__(*args, **kwargs)
        self._processor = processor_cls()
        self._subscribing_msg_type = subscribing_msg_type
        rospy.Subscriber(
            subscribing_topic,
            subscribing_msg,
            self.process,
            queue_size=1,
        )

    def process(self, data):
        if issubclass(self._subscribing_msg_type, List):
            data = data.items

        try:
            output = self._processor(data)
        except NoDataAvailable:
            return

        self.publish(output)
