from iawake.core.contrib.opencv import OpenCVCameraFeed
from iawake.core.service import SerialService
from iawake.face_detection.processor import FaceDetectionProcessor


class FaceDetectionService(SerialService):
    feeds = [OpenCVCameraFeed]
    processor_chain = [FaceDetectionProcessor]


if __name__ == '__main__':
    for result in FaceDetectionService.run(skip_empty=True):
        print result
