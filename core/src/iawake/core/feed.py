class Feed(object):
    published_topic = NotImplemented

    def get_data(self):
        raise NotImplementedError

    def __call__(self):
        return self.get_data()
