#!/usr/bin/env python

from iawake.core.utils import load_module_member


def run_processor():
    node_cls = '{{ node_cls }}'
    node_cls = load_module_member(node_cls)
    node_name = '{{ node_name }}'
    subscribing_topic = '{{ subscribing_topic }}'
    subscribing_msg_module = '{{ subscribing_msg_module }}'
    subscribing_msg = load_module_member(subscribing_msg_module)
    processor_module = '{{ processor_module }}'
    processor_cls = load_module_member(processor_module)
    publishing_topic = '{{ publishing_topic }}'
    publishing_msg_module = '{{ publishing_msg_module }}'
    publishing_msg = load_module_member(publishing_msg_module)
    processor = node_cls(
        subscribing_topic,
        subscribing_msg,
        processor_cls,
        node_name,
        publishing_topic,
        publishing_msg,
    )
    processor.run()


if __name__ == '__main__':
    run_processor()
