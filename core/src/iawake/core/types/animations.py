from schematics.types import FloatType

from iawake.core.types import ReturnType


class Animation(ReturnType):
    pass


class HeadTilt(Animation):
    angle = FloatType()
