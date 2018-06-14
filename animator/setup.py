from setuptools import (
    find_packages,
    setup,
)

core_link = 'git+https://github.com/Automa-Cognoscenti/libs.git' \
            '#egg=iawake.core-0.0.0' \
            '#subdirectory=core'

brain_link = 'git+https://github.com/Automa-Cognoscenti/libs.git' \
             '#egg=iawake.brain-0.0.0' \
             '#subdirectory=brain'

ros_utils_link = 'git+https://github.com/Automa-Cognoscenti/libs.git' \
                 '#egg=iawake.ros_utils-0.0.0' \
                 '#subdirectory=ros-utils'

setup(
    name='iawake.animator',
    version='0.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    dependency_links=[
        core_link,
        brain_link,
        ros_utils_link,
    ],
    install_requires=[
        'iawake.brain==0.0.0',
        'iawake.core==0.0.0',
        'iawake.ros_utils==0.0.0',
    ],
)

