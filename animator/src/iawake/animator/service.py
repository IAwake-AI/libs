from iawake.animator.processor import AnimatorProcessor
from iawake.brain.service import PassthroughBrainService
from iawake.core.service import SerialService


class AnimatorService(SerialService):
    feeds = [PassthroughBrainService]
    processor_chain = [AnimatorProcessor]


if __name__ == '__main__':
    for result in AnimatorService.run(skip_empty=True):
        print result
