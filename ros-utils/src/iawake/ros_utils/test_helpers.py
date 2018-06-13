from iawake.ros_utils.node.base import Node


def generate_serial_publisher_processor(processor_cls):
    class RosPublisherProcessorMeta(type):
        def __new__(mcs, name, bases, namespace):
            return super(RosPublisherProcessorMeta, mcs).__new__(
                mcs,
                processor_cls.__name__ + '_' + name,
                bases,
                namespace,
            )

    class RosPublisherProcessor(processor_cls):
        __metaclass__ = RosPublisherProcessorMeta

        def __init__(self, *args, **kwargs):
            super(processor_cls, self).__init__(*args, **kwargs)
            self._node = Node(
                processor_cls.__name__,
                processor_cls.publishing_topic,
                processor_cls.return_type,
            )

        def process(self, data):
            processed = super(RosPublisherProcessor, self).process(data)
            self._node.publish(processed)
            return processed

    return RosPublisherProcessor
