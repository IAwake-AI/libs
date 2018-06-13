from iawake.emotion.processor import EmotionProcessor

from iawake.core.contrib.opencv import OpenCVCameraFeed
from iawake.core.service import SerialService


class EmotionService(SerialService):
    feeds = [OpenCVCameraFeed]
    processor_chain = [EmotionProcessor]


if __name__ == '__main__':
    for result in EmotionService.run(skip_empty=True):
        print result
