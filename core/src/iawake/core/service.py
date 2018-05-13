from iawake.core.processor import NoDataAvailable


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


class SerialService(Service):
    @classmethod
    def run(cls, skip_empty=False):
        feed = cls._get_feed()
        processor_chain = cls._get_processor_chain()
        while True:
            data = feed()
            for processor in processor_chain:
                try:
                    data = processor(data)
                    assert isinstance(data, processor.return_type)
                except NoDataAvailable as e:
                    if not skip_empty:
                        raise e
                    else:
                        break
            else:
                yield data
