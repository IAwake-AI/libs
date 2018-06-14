import os
import stat
import subprocess

import click
import jinja2
from bs4 import BeautifulSoup

from iawake.core.contrib.opencv import OpenCVCameraFeed, Image
from iawake.core.service import Service
from iawake.core.types import ReturnType
from iawake.core.types.types import List
from iawake.core.utils import load_module_member
from iawake.ros_utils.utils import convert_camel_case_to_snake_case
from trajectory_msgs.msg import JointTrajectory


@click.command()
@click.argument('service_class')
@click.argument('workspace_directory')
@click.option('--pkg_name')
@click.option('--external_usb_cam', is_flag=True)
def generate_ros_files_for_serial_service(
        service_class,
        workspace_directory,
        pkg_name=None,
        external_usb_cam=False,
):
    if pkg_name is None:
        pkg_name = 'iawake__generated'

    templates_dir = os.path.join(os.path.dirname(__file__), 'templates')
    jinja_env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(templates_dir),
    )

    pkg_dir = os.path.join(workspace_directory, 'src', pkg_name)
    pkg_src_dir = os.path.join(pkg_dir, 'src')
    if not os.path.exists(pkg_src_dir):
        os.makedirs(pkg_src_dir)

    launch_document = BeautifulSoup(features='xml')
    launch_document_root = launch_document.new_tag('launch')
    launch_document.append(launch_document_root)

    custom_msgs = []
    _generate_ros_files_for_serial_service(
        service_class,
        workspace_directory,
        launch_document,
        launch_document_root,
        pkg_dir,
        pkg_src_dir,
        pkg_name,
        custom_msgs,
        jinja_env,
        external_usb_cam,
    )

    launch_dir = os.path.join(workspace_directory, 'src/launch')
    if not os.path.exists(launch_dir):
        os.makedirs(launch_dir)

    launch_xml = launch_document.prettify()
    with open("{}/generated.launch".format(launch_dir), 'w') as f:
        f.write(launch_xml)

    cmake_lists_template = jinja_env.get_template('CMakeLists.txt')
    cmake_lists_txt = cmake_lists_template.render(
        pkg_name=pkg_name,
        msg_files='\n'.join(custom_msgs),
    )

    package_xml_template = jinja_env.get_template('package.xml')
    package_xml = package_xml_template.render(pkg_name=pkg_name)

    if not os.path.exists(pkg_src_dir):
        os.makedirs(pkg_src_dir)

    with open("{}/CMakeLists.txt".format(pkg_dir), 'w') as f:
        f.write(cmake_lists_txt)

    with open("{}/package.xml".format(pkg_dir), 'w') as f:
        f.write(package_xml)


def _generate_msg_file_text_from_return_type(return_type):
    msg_file_lines = []
    for field_name, field_type in return_type.fields.items():
        msg = {
            int: 'int32',
            float: 'float64',
            str: 'string',
            unicode: 'string',
        }[field_type.primitive_type]
        msg_file_lines.append(' '.join([msg, field_name]))
    return '\n'.join(msg_file_lines) + '\n'


def _generate_ros_files_for_serial_service(
        service_class,
        workspace_directory,
        launch_document,
        launch_document_root,
        pkg_dir,
        pkg_src_dir,
        pkg_name,
        custom_msgs,
        jinja_env,
        external_usb_cam,
):
    if type(service_class) in (str, unicode):
        service_class = load_module_member(service_class)

    msg_dir = os.path.join(pkg_dir, 'msg')
    custom_msg_files = {}

    def _get_or_setup_publishing_msg_module(return_type):
        if issubclass(return_type, ReturnType):
            if issubclass(return_type, List):
                return_type = return_type.return_type
                module = _get_or_setup_publishing_msg_module(return_type)
                item_name = module.split('.')[-1]
                msg_text = "{}[] items\n".format(item_name)
                type_name = item_name + 'List'
            else:
                type_name = return_type.__name__
                msg_text = None

            _file_name = type_name + '.msg'
            if _file_name not in custom_msgs:
                if msg_text is None:
                    msg_text = _generate_msg_file_text_from_return_type(
                        return_type
                    )
                custom_msgs.append(_file_name)
                _file_path = "{}/{}".format(msg_dir, _file_name)
                custom_msg_files[_file_path] = msg_text
            return "{}.msg.{}".format(pkg_name, type_name)
        else:
            return {
                int: 'std_msgs.msg.Int32',
                str: 'std_msgs.msg.String',
                JointTrajectory: 'trajectory_msgs.msg.JointTrajectory',
            }[return_type]

    node_files = {}
    assert len(service_class.feeds) == 1
    feed = service_class.feeds[0]
    if feed == OpenCVCameraFeed:
        if not external_usb_cam:
            usb_cam_directory = os.path.join(workspace_directory, 'src/usb_cam')
            if not os.path.exists(usb_cam_directory):
                subprocess.check_call([
                    'git',
                    'clone',
                    'https://github.com/ros-drivers/usb_cam',
                    os.path.join(usb_cam_directory, 'usb_cam'),
                ])

            feed_node = _generate_opencv_feed_node_xml(feed, launch_document)
            launch_document_root.append(feed_node)

        subscribing_topic = '/usb_cam/image_raw'
        subscribing_msg_module = 'sensor_msgs.msg.Image'
        subscribing_msg_type = Image.__module__ + '.' + Image.__name__
        processor_node_cls = 'iawake.ros_utils.node.opencv.OpenCVFeedProcessorNode'
    elif issubclass(feed, Service):
        (
            processor_node_cls,
            subscribing_topic,
            subscribing_msg_module,
            subscribing_msg_type,
        ) = _generate_ros_files_for_serial_service(
            feed,
            workspace_directory,
            launch_document,
            launch_document_root,
            pkg_dir,
            pkg_src_dir,
            pkg_name,
            custom_msgs,
            jinja_env,
            external_usb_cam,
        )
    else:
        feed_template = jinja_env.get_template('feed.py')
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
        feed_node = _generate_node_tag(launch_document, feed, pkg_name)
        launch_document_root.append(feed_node)

        subscribing_topic = publishing_topic
        subscribing_msg_module = publishing_msg_module
        subscribing_msg_type = feed.return_type.__module__ + '.' + feed.return_type.__name__
        processor_node_cls = 'iawake.ros_utils.node.processor.ProcessorNode'

    processor_template = jinja_env.get_template('processor.py')
    for processor in service_class.processor_chain:
        node_name = convert_camel_case_to_snake_case(processor.__name__)
        publishing_topic = processor.publishing_topic
        if publishing_topic is None:
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
            subscribing_msg_type=subscribing_msg_type,
            processor_module=processor.__module__ + '.' + processor.__name__,
            publishing_topic=publishing_topic,
            publishing_msg_module=publishing_msg_module,
        )

        processor_node = _generate_node_tag(launch_document, processor, pkg_name)
        launch_document_root.append(processor_node)

        processor_node_cls = 'iawake.ros_utils.node.processor.ProcessorNode'
        subscribing_topic = publishing_topic
        subscribing_msg_module = publishing_msg_module
        subscribing_msg_type = processor.return_type.__module__ + '.' + processor.return_type.__name__

    if len(custom_msg_files) != 0:
        if not os.path.exists(msg_dir):
            os.makedirs(msg_dir)

    for file_name, msg_file_text in custom_msg_files.items():
        with open(file_name, 'w') as f:
            f.write(msg_file_text)

    for file_name, rendered in node_files.items():
        with open(file_name, 'w') as f:
            f.write(rendered)
        st = os.stat(file_name)
        os.chmod(file_name, st.st_mode | stat.S_IEXEC)

    return (
        processor_node_cls,
        subscribing_topic,
        subscribing_msg_module,
        subscribing_msg_type,
    )


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


if __name__ == '__main__':
    _generate_ros_files_for_serial_service('iawake.muse.test_helpers.service.MuseFileService', '.')
