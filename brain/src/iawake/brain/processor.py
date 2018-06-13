from iawake.core.processor import Processor
from iawake.core.types.animations import Animation


class BrainProcessor(Processor):
    return_type = Animation

    def process(self, data):
        raise NotImplementedError


class PassthroughBrainProcessor(BrainProcessor):
    return_type = Animation

    def process(self, data):
        return data
