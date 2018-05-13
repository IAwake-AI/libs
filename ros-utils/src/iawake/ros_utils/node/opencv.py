from cv_bridge import CvBridge

from iawake.ros_utils.node.processor import ProcessorNode


class OpenCVFeedProcessorNode(ProcessorNode):
    def __init__(self, *args, **kwargs):
        super(OpenCVFeedProcessorNode, self).__init__(*args, **kwargs)
        self._bridge = CvBridge()

    def process(self, data):
        image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        return super(OpenCVFeedProcessorNode, self).process(image)
