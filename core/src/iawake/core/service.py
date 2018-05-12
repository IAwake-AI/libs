from iawake.core.processor import NoDataAvailable


class Service(object):
    feed_cls = NotImplemented
    processor_chain_classes = NotImplemented

    @classmethod
    def get_feed(cls):
        if cls.feed_cls is NotImplemented:
            raise NotImplementedError
        return cls.feed_cls()

    @classmethod
    def get_processor_chain(cls):
        if cls.processor_chain_classes is NotImplemented:
            raise NotImplementedError
        return [
            processor_cls()
            for processor_cls in cls.processor_chain_classes
        ]

    @classmethod
    def run(cls):
        raise NotImplementedError


class SerialService(Service):
    @classmethod
    def run(cls, skip_empty=False):
        feed = cls.get_feed()
        processor_chain = cls.get_processor_chain()
        while True:
            data = feed()
            for processor in processor_chain:
                try:
                    data = processor(data)
                except NoDataAvailable as e:
                    if not skip_empty:
                        raise e
                    else:
                        break
            else:
                yield data
