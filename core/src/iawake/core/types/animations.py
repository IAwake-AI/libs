import json

from schematics import Model
from schematics.types import (
    FloatType,
    StringType,
)

from iawake.core.types import ReturnType


class AnimationWrapper(ReturnType):
    name = StringType()
    data = StringType()

    @classmethod
    def from_animation(cls, movement):
        return cls({
            'name': type(movement).__name__,
            'data': json.dumps(movement.to_primitive()),
        })


class HeadTilt(Model):
    angle = FloatType()


def get_animation_from_wrapper(wrapper):
    cls = {
        HeadTilt.__name__: HeadTilt,
    }[wrapper.name]
    data = json.loads(wrapper.data)
    return cls(data)
