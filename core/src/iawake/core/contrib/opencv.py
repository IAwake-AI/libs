import cv2

from iawake.core.feed import Feed


class OpenCVCameraFeed(Feed):
    def __init__(self, camera_index=0):
        self._capture = cv2.VideoCapture(camera_index)

    def get_data(self):
        ret, img = self._capture.read()
        return img
