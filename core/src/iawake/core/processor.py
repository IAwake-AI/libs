from iawake.core.utils import validate_service_element_response


class NoDataAvailable(Exception):
    pass


class Processor(object):
    return_type = NotImplemented
    publishing_topic = None

    def process(self, data):
        raise NotImplementedError

    def validate_response(self, response):
        return validate_service_element_response(self, response)

    def __call__(self, data):
        return self.process(data)
