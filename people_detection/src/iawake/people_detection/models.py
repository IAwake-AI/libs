from schematics import Model
from schematics.types import (
    FloatType,
    IntType,
    ModelType,
)


class BoundingBox(Model):
    x = IntType()
    y = IntType()
    w = IntType()
    h = IntType()


class DetectedPerson(Model):
    bounding_box = ModelType(BoundingBox)
    confidence = FloatType(min_value=0, max_value=1)
