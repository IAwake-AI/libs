import cv2

from iawake.core.feed import Feed


class OpenCVCameraFeed(Feed):
    camera_index = 0

    def __init__(self):
        self._capture = cv2.VideoCapture(self.camera_index)

    def get_data(self):
        ret, img = self._capture.read()
        return img
