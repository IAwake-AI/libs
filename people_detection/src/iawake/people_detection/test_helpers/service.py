from iawake.core.contrib.test_helpers \
    import generate_debug_bounding_box_display_processor
from iawake.people_detection.processor import PeopleToRectanglesProcessor
from iawake.people_detection.service import PeopleDetectionService
from iawake.people_detection.test_helpers.feed \
    import PeopleDetectionServiceStaticVideoFeed


class StaticVideoFeedPeopleTrackingService(PeopleDetectionService):
    feed = PeopleDetectionServiceStaticVideoFeed


DebugProcessor = generate_debug_bounding_box_display_processor(
    PeopleToRectanglesProcessor
)


class DebugStaticVideoFeedPeopleTrackingService(StaticVideoFeedPeopleTrackingService):
    processor_chain = [DebugProcessor]


if __name__ == '__main__':
    for result in DebugStaticVideoFeedPeopleTrackingService.run(skip_empty=True):
        print result
