class NoDataAvailable(Exception):
    pass


class Processor(object):
    return_type = NotImplemented

    def process(self, data):
        raise NotImplementedError

    def __call__(self, data):
        return self.process(data)
