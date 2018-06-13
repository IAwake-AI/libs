from iawake.core.service import SerialService
from iawake.muse.feed import MuseFeed


class MuseService(SerialService):
    feed = MuseFeed
    processor_chain = []


if __name__ == '__main__':
    for result in MuseService.run(skip_empty=True):
        print result
