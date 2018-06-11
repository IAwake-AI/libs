import os

from iawake.core.feed import FileFeed
from iawake.muse.feed import MuseFeed
from iawake.muse.types import MuseReturnType


class MuseFileFeed(FileFeed, MuseFeed):
    @property
    def _file_path(self):
        data_dir = os.path.join(os.path.dirname(__file__), 'data')
        return "{}/test_data.txt".format(data_dir)

    @classmethod
    def _process_line(cls, line):
        values = line.split(',')
        response = MuseReturnType({
            'TP9': values[0],
            'Fp1': values[1],
            'Fp2': values[2],
            'TP10': values[3],
        })
        return response
