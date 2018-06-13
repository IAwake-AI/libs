from iawake.ros_utils.node.base import Node


def generate_serial_publisher_processor(processor_cls):
    class RosPublisherProcessor(processor_cls):
        def __init__(self, *args, **kwargs):
            super(RosPublisherProcessor, self).__init__(*args, **kwargs)
            self._node = Node(
                processor_cls.__name__,
                processor_cls.publishing_topic,
                processor_cls.return_type,
            )

        def process(self, data):
            processed = super(RosPublisherProcessor, self).process(data)
            self._node.publish(processed)

    return RosPublisherProcessor
