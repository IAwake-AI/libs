from iawake.core.feed import Feed
from iawake.core.processor import NoDataAvailable
from iawake.core.types import ReturnType


class Service(Feed):
    feeds = NotImplemented
    processor_chain = NotImplemented

    @property
    def return_type(self):
        return self.processor_chain[-1].return_type

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


class SerialService(Service):
    def __init__(self):
        self._feeds = self._get_feeds()
        self._processor_chain = self._get_processor_chain()
        self._feed_index = 0

    def get_data(self, strict_return_type=False):
        feed = self._feeds[self._feed_index]
        self._feed_index += 1
        if self._feed_index >= len(self._feeds):
            self._feed_index = 0

        data = feed.get_data(strict_return_type=strict_return_type)
        if strict_return_type:
            feed.validate_response(data)
        for processor in self._processor_chain:
            data = processor(data)
            if strict_return_type:
                processor.validate_response(data)
        else:
            return data

    @classmethod
    def run(cls, skip_empty=False, strict_return_type=False):
        instance = cls()
        while True:
            try:
                yield instance.get_data(strict_return_type)
            except NoDataAvailable:
                if not skip_empty:
                    raise
                else:
                    continue
