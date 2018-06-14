from iawake.core.processor import Processor
from iawake.core.types.animations import AnimationWrapper


class BrainProcessor(Processor):
    return_type = AnimationWrapper

    def process(self, data):
        raise NotImplementedError


class PassthroughBrainProcessor(BrainProcessor):
    def process(self, data):
        return data
