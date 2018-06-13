from iawake.face_detection.service import FaceDetectionService
from iawake.face_detection.test_helpers.feed import FaceDetectionStaticVideoFeed
from iawake.face_detection.test_helpers.processor \
    import DebugDisplayFaceDetectionProcessor


class DebugDisplayFaceDetectionService(FaceDetectionService):
    processor_chain = [DebugDisplayFaceDetectionProcessor]


class StaticVideoDebugDisplayFaceDetectionService(
    DebugDisplayFaceDetectionService
):
    feeds = [FaceDetectionStaticVideoFeed]
    processor_chain = [DebugDisplayFaceDetectionProcessor]


if __name__ == '__main__':
    for result in StaticVideoDebugDisplayFaceDetectionService.run(
            skip_empty=True
    ):
        print result
