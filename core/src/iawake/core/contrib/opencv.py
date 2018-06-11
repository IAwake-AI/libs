import cv2

from iawake.core.feed import (
    DirectoryFeed,
    Feed,
)


class OpenCVCameraFeed(Feed):
    camera_index = 0

    def __init__(self):
        self._capture = cv2.VideoCapture(self.camera_index)

    def get_data(self):
        ret, img = self._capture.read()
        return img


class OpenCVStaticVideoFeed(DirectoryFeed):
    @classmethod
    def _process_file(cls, path):
        return cv2.imread(path)
