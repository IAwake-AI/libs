from iawake.core.feed import Feed
from iawake.muse.types import MuseReturnType


class MuseFeed(Feed):
    return_type = MuseReturnType

    def get_data(self):
        raise NotImplementedError
