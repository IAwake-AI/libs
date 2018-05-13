from iawake.core.processor import NoDataAvailable
from iawake.core.types import ReturnType


class Service(object):
    feed = NotImplemented
    processor_chain = NotImplemented

    @classmethod
    def _get_feed(cls):
        if cls.feed is NotImplemented:
            raise NotImplementedError
        return cls.feed()

    @classmethod
    def _get_processor_chain(cls):
        if cls.processor_chain is NotImplemented:
            raise NotImplementedError
        return [
            processor()
            for processor in cls.processor_chain
        ]

    @classmethod
    def run(cls):
        raise NotImplementedError


def _validate_response(response, expected_return_type):
    assert isinstance(response, expected_return_type)
    if isinstance(expected_return_type, ReturnType):
        response.validate()


class SerialService(Service):
    @classmethod
    def run(cls, skip_empty=False, strict_return_type=False):
        feed = cls._get_feed()
        processor_chain = cls._get_processor_chain()
        while True:
            data = feed()
            if strict_return_type:
                _validate_response(data, feed.return_type)
            for processor in processor_chain:
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
                yield data
