import os

from iawake.core.contrib.opencv import OpenCVStaticVideoFeed


class PeopleDetectionServiceStaticVideoFeed(OpenCVStaticVideoFeed):
    @property
    def _directory_path(self):
        return os.path.join(os.path.dirname(__file__), 'data')
