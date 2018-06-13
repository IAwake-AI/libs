from iawake.muse.service import MuseService
from iawake.muse.test_helpers.feed import MuseFileFeed


class MuseFileService(MuseService):
    feeds = [MuseFileFeed]


if __name__ == '__main__':
    for result in MuseFileService.run(skip_empty=True):
        print result.to_primitive()
