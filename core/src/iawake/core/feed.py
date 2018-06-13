import os

from iawake.core.utils import validate_service_element_response


class Feed(object):
    return_type = NotImplemented

    def get_data(self, **kwargs):
        raise NotImplementedError

    def validate_response(self, response):
        return validate_service_element_response(self, response)

    def __call__(self):
        return self.get_data()


class FileFeed(Feed):
    _file_path = NotImplemented

    def __init__(self):
        self._file = open(self._file_path)

    def _get_next_line(self):
        line = self._file.readline()
        if line == '':
            self._file.seek(0)
            line = self._file.readline()
        return line.rstrip()

    @classmethod
    def _process_line(cls, line):
        raise NotImplementedError

    def get_data(self, **kwargs):
        line = self._get_next_line()
        return self._process_line(line)


class DirectoryFeed(Feed):
    _directory_path = NotImplemented

    def __init__(self):
        self._files = sorted(os.listdir(self._directory_path))
        self._index = 0

    def _get_next_path(self):
        path = os.path.join(self._directory_path, self._files[self._index])
        self._index += 1
        if self._index >= len(self._files):
            self._index = 0
        return path

    @classmethod
    def _process_file(cls, path):
        raise NotImplementedError

    def get_data(self, **kwargs):
        path = self._get_next_path()
        return self._process_file(path)
