from iawake.brain.processor import PassthroughBrainProcessor
from iawake.core.service import SerialService
from iawake.face_tracking.service import FaceTrackingService


class PassthroughBrainService(SerialService):
    feeds = [FaceTrackingService]
    processor_chain = [PassthroughBrainProcessor]


if __name__ == '__main__':
    for result in PassthroughBrainService.run(skip_empty=True):
        print result
