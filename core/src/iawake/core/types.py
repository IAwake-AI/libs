from schematics import Model
from schematics.types import (
    FloatType,
    IntType,
)


class ReturnType(Model):
    pass


class DetectedBox(ReturnType):
    x = IntType()
    y = IntType()
    w = IntType()
    h = IntType()
    confidence = FloatType()
