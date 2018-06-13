from iawake.core.feed import Feed
from iawake.core.processor import NoDataAvailable
from iawake.core.types import ReturnType


class Service(Feed):
    feeds = NotImplemented
    processor_chain = NotImplemented

    @classmethod
    def _get_feeds(cls):
        if cls.feeds is NotImplemented:
            raise NotImplementedError
        return [feed() for feed in cls.feeds]

    @classmethod
    def _get_processor_chain(cls):
        if cls.processor_chain is NotImplemented:
            raise NotImplementedError
        return [
            processor()
            for processor in cls.processor_chain
        ]

    def get_data(self):
        raise NotImplementedError

    def run(self):
        raise NotImplementedError


def _validate_response(response, expected_return_type):
    assert isinstance(response, expected_return_type)
    if isinstance(expected_return_type, ReturnType):
        response.validate()


class SerialService(Service):
    def __init__(self):
        self._feeds = self._get_feeds()
        self._processor_chain = self._get_processor_chain()
        self._feed_index = 0

    def get_data(self, skip_empty=False, strict_return_type=False):
        feed = self._feeds[self._feed_index]
        data = feed()
        self._feed_index += 1
        if self._feed_index >= len(self._feeds):
            self._feed_index = 0

        if strict_return_type:
            _validate_response(data, feed.return_type)
        for processor in self._processor_chain:
            try:
                data = processor(data)
                if strict_return_type:
                    _validate_response(data, processor.return_type)
            except NoDataAvailable as e:
                if not skip_empty:
                    raise e
                else:
                    break
        else:
            return data

    @classmethod
    def run(cls, skip_empty=False, strict_return_type=False):
        instance = cls()
        while True:
            yield instance.get_data(skip_empty, strict_return_type)
