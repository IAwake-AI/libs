from schematics import Model
from schematics.types import (
    FloatType,
    IntType,
)


class ReturnType(Model):
    @classmethod
    def validate_response(cls, response):
        if not isinstance(response, cls):
            return False
        response.validate()
        return True


class List(ReturnType):
    def __init__(self, return_type):
        super(List, self).__init__()
        self.return_type = return_type


class DetectedBox(ReturnType):
    x = IntType()
    y = IntType()
    w = IntType()
    h = IntType()
    confidence = FloatType()
