from iawake.face_detection.test_helpers.service \
    import StaticVideoFaceDetectionService
from iawake.face_tracking.service import FaceTrackingService


class StaticVideoFaceTrackingService(FaceTrackingService):
    feeds = [StaticVideoFaceDetectionService]


if __name__ == '__main__':
    for result in StaticVideoFaceTrackingService.run(skip_empty=True):
        print result.to_primitive()
