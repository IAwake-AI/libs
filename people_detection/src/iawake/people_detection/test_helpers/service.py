from iawake.people_detection.service import PeopleDetectionService
from iawake.people_detection.test_helpers.feed \
    import PeopleDetectionServiceStaticVideoFeed
from iawake.people_detection.test_helpers.processor \
    import DebugDisplayPeopleToRectanglesProcessor


class StaticVideoFeedPeopleTrackingService(PeopleDetectionService):
    feed = PeopleDetectionServiceStaticVideoFeed


class DebugStaticVideoFeedPeopleTrackingService(StaticVideoFeedPeopleTrackingService):
    processor_chain = [DebugDisplayPeopleToRectanglesProcessor]


if __name__ == '__main__':
    for result in DebugStaticVideoFeedPeopleTrackingService.run(skip_empty=True):
        print result
