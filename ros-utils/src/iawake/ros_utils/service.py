import os
import stat
import subprocess

import click
import jinja2
from bs4 import BeautifulSoup

from iawake.core.contrib.opencv import OpenCVCameraFeed
from iawake.core.types import ReturnType
from iawake.core.utils import load_module_member
from iawake.ros_utils.utils import convert_camel_case_to_snake_case

from trajectory_msgs.msg import JointTrajectory


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
        pkg_name = 'iawake__generated'
    return _generate_ros_files_for_serial_service(
        service_class,
        workspace_directory,
        pkg_name,
    )


def _generate_msg_file_text_from_return_type(return_type):
    msg_file_lines = []
    for field_name, field_type in return_type.fields.items():
        msg = {
            int: 'int32',
            float: 'float64',
            str: 'string',
        }[field_type.primitive_type]
        msg_file_lines.append(' '.join([msg, field_name]))
    return '\n'.join(msg_file_lines)


def _generate_ros_files_for_serial_service(
        service_class,
        workspace_directory,
        pkg_name='iawake__generated',
):
    if type(service_class) in (str, unicode):
        service_class = load_module_member(service_class)

    pkg_dir = os.path.join(workspace_directory, 'src', pkg_name)
    pkg_src_dir = os.path.join(pkg_dir, 'src')
    launch_xml = _generate_launch_xml(service_class, pkg_name)
    templates_dir = os.path.join(os.path.dirname(__file__), 'templates')
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(templates_dir))

    msg_dir = os.path.join(pkg_dir, 'msg')
    custom_msgs = []
    custom_msg_files = {}

    def _get_or_setup_publishing_msg_module(return_type):
        if issubclass(return_type, ReturnType):
            _file_name = return_type.__name__ + '.msg'
            if _file_name not in custom_msgs:
                msg_text = _generate_msg_file_text_from_return_type(return_type)
                custom_msgs.append(_file_name)
                _file_path = "{}/{}".format(msg_dir, _file_name)
                custom_msg_files[_file_path] = msg_text
            return "{}.msg.{}".format(pkg_name, return_type.__name__)
        else:
            return {
                int: 'std_msgs.msg.Int32',
                str: 'std_msgs.msg.String',
		JointTrajectory: 'trajectory_msgs.msg.JointTrajectory',
            }[return_type]

    node_files = {}
    if service_class.feed == OpenCVCameraFeed:
        usb_cam_directory = os.path.join(workspace_directory, 'src/usb_cam')
        if not os.path.exists(usb_cam_directory):
            subprocess.check_call([
                'git',
                'clone',
                'https://github.com/ros-drivers/usb_cam',
                os.path.join(usb_cam_directory, 'usb_cam'),
            ])
        subscribing_topic = '/usb_cam/image_raw'
        subscribing_msg_module = 'sensor_msgs.msg.Image'
        processor_node_cls = 'iawake.ros_utils.node.opencv.OpenCVFeedProcessorNode'
    else:
        feed = service_class.feed
        feed_template = env.get_template('feed.py')
        node_name = convert_camel_case_to_snake_case(feed.__name__)
        publishing_topic = "{}/{}".format(pkg_name, node_name)
        publishing_msg_module = _get_or_setup_publishing_msg_module(
            feed.return_type
        )
        file_name = "{}/{}.py".format(pkg_src_dir, node_name)
        node_files[file_name] = feed_template.render(
            node_name=node_name,
            feed_module=feed.__module__ + '.' + feed.__name__,
            publishing_topic=publishing_topic,
            publishing_msg_module=publishing_msg_module,
        )
        subscribing_topic = publishing_topic
        subscribing_msg_module = publishing_msg_module
        processor_node_cls = 'iawake.ros_utils.node.processor.ProcessorNode'

    processor_template = env.get_template('processor.py')
    for processor in service_class.processor_chain:
        node_name = convert_camel_case_to_snake_case(processor.__name__)
        publishing_topic = "{}/{}".format(pkg_name, node_name)
        file_name = "{}/{}.py".format(pkg_src_dir, node_name)
        publishing_msg_module = _get_or_setup_publishing_msg_module(
            processor.return_type,
        )
        node_files[file_name] = processor_template.render(
            node_cls=processor_node_cls,
            node_name=node_name,
            subscribing_topic=subscribing_topic,
            subscribing_msg_module=subscribing_msg_module,
            processor_module=processor.__module__ + '.' + processor.__name__,
            publishing_topic=publishing_topic,
            publishing_msg_module=publishing_msg_module,
        )
        processor_node_cls = 'iawake.ros_utils.node.processor.ProcessorNode'
        subscribing_topic = publishing_topic

    cmake_lists_template = env.get_template('CMakeLists.txt')
    cmake_lists_txt = cmake_lists_template.render(
        pkg_name=pkg_name,
        msg_files='\n'.join(custom_msgs),
    )

    package_xml_template = env.get_template('package.xml')
    package_xml = package_xml_template.render(pkg_name=pkg_name)

    launch_dir = os.path.join(workspace_directory, 'src/launch')
    if not os.path.exists(launch_dir):
        os.makedirs(launch_dir)

    if not os.path.exists(pkg_src_dir):
        os.makedirs(pkg_src_dir)

    if len(custom_msg_files) != 0:
        if not os.path.exists(msg_dir):
            os.makedirs(msg_dir)

    for file_name, msg_file_text in custom_msg_files.items():
        with open(file_name, 'w') as f:
            f.write(msg_file_text)

    with open("{}/generated.launch".format(launch_dir), 'w') as f:
        f.write(launch_xml)

    with open("{}/CMakeLists.txt".format(pkg_dir), 'w') as f:
        f.write(cmake_lists_txt)

    with open("{}/package.xml".format(pkg_dir), 'w') as f:
        f.write(package_xml)

    for file_name, rendered in node_files.items():
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


def _generate_node_tag(document, node, pkg_name):
    snake_class_name = convert_camel_case_to_snake_case(
        node.__name__,
    )
    return _generate_named_tag(
        document,
        'node',
        pkg=pkg_name,
        type="{}.py".format(snake_class_name),
        name=snake_class_name
    )


def _generate_launch_xml(service_class, pkg_name):
    document = BeautifulSoup(features='xml')
    root = document.new_tag('launch')
    document.append(root)
    if service_class.feed == OpenCVCameraFeed:
        feed_node = _generate_opencv_feed_node_xml(service_class.feed, document)
    else:
        feed_node = _generate_node_tag(document, service_class.feed, pkg_name)
    root.append(feed_node)
    for processor in service_class.processor_chain:
        root.append(_generate_node_tag(document, processor, pkg_name))
    return document.prettify()


if __name__ == '__main__':
    _generate_ros_files_for_serial_service('iawake.muse.test_helpers.service.MuseFileService', '.')
