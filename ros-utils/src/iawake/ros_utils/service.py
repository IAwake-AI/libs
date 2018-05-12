import os
import stat
import subprocess

import click
import jinja2
from bs4 import BeautifulSoup

from iawake.core.contrib.opencv import OpenCVCameraFeed
from iawake.core.utils import load_module_member
from iawake.ros_utils.utils import convert_camel_case_to_snake_case


@click.command()
@click.argument('service_class')
@click.argument('workspace_directory')
@click.option('--pkg_name')
def generate_ros_files_for_serial_service(
        service_class,
        workspace_directory,
        pkg_name=None,
):
    if pkg_name is None:
        pkg_name = 'iawake'
    return _generate_ros_files_for_serial_service(
        service_class,
        workspace_directory,
        pkg_name,
    )


def _generate_ros_files_for_serial_service(
        service_class,
        workspace_directory,
        pkg_name='iawake',
):
    if type(service_class) in (str, unicode):
        service_class = load_module_member(service_class)

    if service_class.feed != OpenCVCameraFeed:
        raise Exception('unsupported feed')
    else:
        usb_cam_directory = os.path.join(workspace_directory, 'src/usb_cam')
        if not os.path.exists(usb_cam_directory):
            subprocess.check_call([
                'git',
                'clone',
                'https://github.com/ros-drivers/usb_cam',
                os.path.join(usb_cam_directory, 'usb_cam'),
            ])
        subscribing_topic = '/usb_cam/image_raw'
        processor_node_cls = 'iawake.ros_utils.processor_node.opencv.OpenCVFeedProcessorNode'

    pkg_dir = os.path.join(workspace_directory, 'src', pkg_name)
    pkg_src_dir = os.path.join(pkg_dir, 'src')
    launch_xml = _generate_launch_xml(service_class, pkg_name)
    templates_dir = os.path.join(os.path.dirname(__file__), 'templates')
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(templates_dir))
    template = env.get_template('processor.py')
    processor_node_files = {}
    for processor in service_class.processor_chain:
        node_name = convert_camel_case_to_snake_case(processor.__name__)
        publishing_topic = "{}/{}".format(pkg_name, node_name)
        file_name = "{}/{}.py".format(pkg_src_dir, node_name)
        processor_node_files[file_name] = template.render(
            node_cls=processor_node_cls,
            node_name=node_name,
            subscribing_topic=subscribing_topic,
            publishing_topic=publishing_topic,
            processor_module=processor.__module__ + '.' + processor.__name__,
        )
        processor_node_cls = 'iawake.ros_utils.processor_node.base.ProcessorNode'
        subscribing_topic = publishing_topic

    cmake_lists_template = env.get_template('CMakeLists.txt')
    cmake_lists_txt = cmake_lists_template.render(pkg_name=pkg_name)

    package_xml_template = env.get_template('package.xml')
    package_xml = package_xml_template.render(pkg_name=pkg_name)

    launch_dir = os.path.join(workspace_directory, 'src/launch')
    if not os.path.exists(launch_dir):
        os.makedirs(launch_dir)

    if not os.path.exists(pkg_src_dir):
        os.makedirs(pkg_src_dir)

    with open("{}/generated.launch".format(launch_dir), 'w') as f:
        f.write(launch_xml)

    with open("{}/CMakeLists.txt".format(pkg_dir), 'w') as f:
        f.write(cmake_lists_txt)

    with open("{}/package.xml".format(pkg_dir), 'w') as f:
        f.write(package_xml)

    for file_name, rendered in processor_node_files.items():
        with open(file_name, 'w') as f:
            f.write(rendered)
        st = os.stat(file_name)
        os.chmod(file_name, st.st_mode | stat.S_IEXEC)


def _generate_named_tag(document, tag_name, **attrs):
    name = attrs.pop('name')
    tag = document.new_tag(tag_name, **attrs)
    tag.attrs['name'] = name
    return tag


def _generate_opencv_feed_node_xml(feed, document):
    node = _generate_named_tag(
        document,
        'node',
        name='usb_cam',
        pkg='usb_cam',
        type='usb_cam_node',
    )
    video_device = "/dev/video{}".format(feed.camera_index)
    node.append(
        _generate_named_tag(
            document,
            'param',
            name='video_device',
            value=video_device,
        )
    )
    return node


def _generate_launch_xml(service_class, pkg_name):
    document = BeautifulSoup(features='xml')
    root = document.new_tag('launch')
    document.append(root)
    feed_node = _generate_opencv_feed_node_xml(service_class.feed, document)
    root.append(feed_node)
    for processor in service_class.processor_chain:
        snake_class_name = convert_camel_case_to_snake_case(
            processor.__name__,
        )
        node = _generate_named_tag(
            document,
            'node',
            pkg=pkg_name,
            type="{}.py".format(snake_class_name),
            name=snake_class_name
        )
        root.append(node)
    return document.prettify()
