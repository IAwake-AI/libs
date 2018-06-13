from iawake.animator.processor import AnimatorProcessor
from iawake.ros_utils.test_helpers import generate_serial_publisher_processor

SerialPublisherAnimatorProcessor = generate_serial_publisher_processor(
    AnimatorProcessor
)
