from iawake.core.service import SerialService
from iawake.face_detection.service import FaceDetectionService
from iawake.face_tracking.processor import FaceTrackingProcessor


class FaceTrackingService(SerialService):
    feeds = [FaceDetectionService]
    processor_chain = [FaceTrackingProcessor]


if __name__ == '__main__':
    for result in FaceTrackingService.run(skip_empty=True):
        print result
