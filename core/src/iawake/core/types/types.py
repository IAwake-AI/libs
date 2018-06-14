from schematics import Model, ModelMeta
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


class ListMeta(ModelMeta):
    def __call__(cls, return_type):
        cls.return_type = return_type
        return cls


class List(ReturnType):
    __metaclass__ = ListMeta


class DetectedBox(ReturnType):
    x = IntType()
    y = IntType()
    w = IntType()
    h = IntType()
    confidence = FloatType()
