#!/usr/bin/env python

from iawake.core.utils import load_module_member
from iawake.ros_utils.node.feed import FeedNode


def run_feed():
    node_name = '{{ node_name }}'
    feed_module = '{{ feed_module }}'
    feed_cls = load_module_member(feed_module)
    publishing_topic = '{{ publishing_topic }}'
    publishing_msg_module = '{{ publishing_msg_module }}'
    publishing_msg = load_module_member(publishing_msg_module)
    processor = FeedNode(
        feed_cls,
        node_name,
        publishing_topic,
        publishing_msg,
    )
    processor.run()


if __name__ == '__main__':
    run_feed()
