from iawake.animator.service import AnimatorService
from iawake.animator.test_helpers.processor \
    import SerialPublisherAnimatorProcessor
from iawake.brain.test_helpers.service import StaticPassthroughBrainService


class StaticPassthroughBrainAnimatorService(AnimatorService):
    feeds = [StaticPassthroughBrainService]
    processor_chain = [SerialPublisherAnimatorProcessor]


if __name__ == '__main__':
    for result in StaticPassthroughBrainAnimatorService.run(
            skip_empty=True,
            strict_return_type=True,
    ):
        pass
