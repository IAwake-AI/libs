import os

from iawake.muse.feed import MuseFeed
from iawake.muse.types import MuseReturnType


class MuseFileFeed(MuseFeed):
    def __init__(self):
        data_dir = os.path.join(os.path.dirname(__file__), 'data')
        self._file = open("{}/test_data.txt".format(data_dir))

    def _get_next_line(self):
        line = self._file.readline()
        if line == '':
            self._file.seek(0)
            line = self._file.readline()
        return line.rstrip()

    def get_data(self):
        line = self._get_next_line()
        values = line.split(',')
        response = MuseReturnType({
            'TP9': values[0],
            'Fp1': values[1],
            'Fp2': values[2],
            'TP10': values[3],
        })
        return response
