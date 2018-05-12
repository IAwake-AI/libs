from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from iawake.ros_utils.processor_node.base import ProcessorNode


class OpenCVFeedProcessorNode(ProcessorNode):
    subscribing_msg = Image

    def __init__(self, *args, **kwargs):
        super(OpenCVFeedProcessorNode, self).__init__(*args, **kwargs)
        self._bridge = CvBridge()

    def process(self, data):
        image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        return super(OpenCVFeedProcessorNode, self).process(image)
