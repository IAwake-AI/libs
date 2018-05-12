#!/usr/bin/env python

from iawake.core.utils import load_module_member


def run_processor():
    node_cls = '{{ node_cls }}'
    node_cls = load_module_member(node_cls)
    node_name = '{{ node_name }}'
    subscribing_topic = '{{ subscribing_topic }}'
    processor_module = '{{ processor_module }}'
    publishing_topic = '{{ publishing_topic }}'
    processor = node_cls(
        node_name,
        subscribing_topic,
        publishing_topic,
        processor_module,
    )
    processor.run()


if __name__ == '__main__':
    run_processor()
