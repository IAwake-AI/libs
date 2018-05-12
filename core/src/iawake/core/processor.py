class NoDataAvailable(Exception):
    pass


class Processor(object):
    def process(self, data):
        raise NotImplementedError

    def __call__(self, data):
        return self.process(data)
