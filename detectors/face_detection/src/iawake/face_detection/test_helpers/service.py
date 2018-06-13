from iawake.face_detection.service import FaceDetectionService
from iawake.face_detection.test_helpers.feed import FaceDetectionStaticVideoFeed
from iawake.face_detection.test_helpers.processor \
    import DebugDisplayFaceDetectionProcessor


class StaticVideoFaceDetectionService(FaceDetectionService):
    feeds = [FaceDetectionStaticVideoFeed]


class StaticVideoDebugDisplayFaceDetectionService(
    StaticVideoFaceDetectionService
):
    processor_chain = [DebugDisplayFaceDetectionProcessor]


if __name__ == '__main__':
    for result in StaticVideoDebugDisplayFaceDetectionService.run(
            skip_empty=True
    ):
        print result
