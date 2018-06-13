import cv2
import numpy

from iawake.core.feed import (
    DirectoryFeed,
    Feed,
)
from iawake.core.types.types import ReturnType


class Image(ReturnType):
    @classmethod
    def validate_response(cls, response):
        return isinstance(response, numpy.ndarray)


class OpenCVCameraFeed(Feed):
    camera_index = 0
    return_type = Image

    def __init__(self):
        self._capture = cv2.VideoCapture(self.camera_index)

    def get_data(self, **kwargs):
        ret, img = self._capture.read()
        return img


class OpenCVStaticVideoFeed(DirectoryFeed):
    return_type = Image

    @classmethod
    def _process_file(cls, path):
        return cv2.imread(path)
