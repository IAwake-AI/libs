from iawake.emotion.processor import EmotionProcessor

from iawake.core.contrib.opencv import OpenCVCameraFeed
from iawake.core.service import SerialService


class EmotionService(SerialService):
    feed_cls = OpenCVCameraFeed
    processor_chain_classes = [EmotionProcessor]


if __name__ == '__main__':
    for result in EmotionService.run(skip_empty=True):
        print result
