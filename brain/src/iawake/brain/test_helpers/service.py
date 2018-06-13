from iawake.brain.service import PassthroughBrainService
from iawake.face_tracking.test_helpers.service \
    import StaticVideoFaceTrackingService


class StaticPassthroughBrainService(PassthroughBrainService):
    feeds = [StaticVideoFaceTrackingService]


if __name__ == '__main__':
    for result in StaticPassthroughBrainService.run(skip_empty=True):
        print result
