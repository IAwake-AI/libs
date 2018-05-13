from iawake.face_tracker.processor import FaceProcessor
from iawake.face_tracker.processor import RobotProcessor

from iawake.core.contrib.opencv import OpenCVCameraFeed
from iawake.core.service import SerialService


class FaceService(SerialService):
    feed = OpenCVCameraFeed
    processor_chain = [FaceProcessor,RobotProcessor]


if __name__ == '__main__':
    for result in FaceService.run(skip_empty=True):
        print result
