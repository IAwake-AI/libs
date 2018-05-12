from setuptools import (
    find_packages,
    setup,
)

setup(
    name='iawake.ros_utils',
    version='0.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    dependency_links=[
        'git+https://github.com/Automa-Cognoscenti/libs.git#egg=iawake.core-0.0.0#subdirectory=core',
    ],
    install_requires=[
        'beautifulsoup4==4.6.0',
        'catkin_pkg==0.4.2',
        'click==6.7',
        'iawake.core[opencv-python]==0.0.0',
        'jinja2==2.10',
        'lxml==4.2.1',
        'rospkg==1.1.4',
    ],
    entry_points={
        'console_scripts': [
            'setup_ros_for_service = iawake.ros_utils.service:generate_ros_files_for_serial_service',
        ]
    },
)
