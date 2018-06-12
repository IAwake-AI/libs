from iawake.core.contrib.opencv import OpenCVCameraFeed
from iawake.core.service import SerialService
from iawake.people_detection.processor import PeopleToRectanglesProcessor


class PeopleDetectionService(SerialService):
    feed = OpenCVCameraFeed
    processor_chain = [PeopleToRectanglesProcessor]


if __name__ == '__main__':
    for result in PeopleDetectionService.run(skip_empty=True):
        print result
