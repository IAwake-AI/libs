from setuptools import (
    find_packages,
    setup,
)

core_link = 'git+https://github.com/Automa-Cognoscenti/libs.git' \
            '#egg=iawake.core-0.0.0' \
            '#subdirectory=core'

face_tracking_link = 'git+https://github.com/Automa-Cognoscenti/libs.git' \
                     '#egg=iawake.face_tracking-0.0.0' \
                     '#subdirectory=deciders/face_tracking'

setup(
    name='iawake.brain',
    version='0.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    dependency_links=[
        core_link,
        face_tracking_link,
    ],
    install_requires=[
        'iawake.core==0.0.0',
        'iawake.face_tracking==0.0.0',
    ],
)
