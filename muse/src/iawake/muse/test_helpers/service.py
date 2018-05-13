from iawake.muse.service import MuseService
from iawake.muse.test_helpers.feed import MuseFileFeed


class MuseFileService(MuseService):
    feed = MuseFileFeed


if __name__ == '__main__':
    for result in MuseFileService.run():
        print result
        print 3
