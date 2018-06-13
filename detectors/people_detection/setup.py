from setuptools import (
    find_packages,
    setup,
)

setup(
    name='iawake.people_detection',
    version='0.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    dependency_links=[
        'git+https://github.com/Automa-Cognoscenti/libs.git#egg=iawake.core-0.0.0#subdirectory=core',
    ],
    install_requires=[
        'iawake.core[opencv]==0.0.0',
        'imutils==0.4.6',
        'opencv-python==3.4.0.12',
        'schematics==2.0.1',
    ],
)