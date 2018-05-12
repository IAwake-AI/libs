from setuptools import (
    find_packages,
    setup,
)

setup(
    name='iawake.core',
    version='0.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    extras_require={
        'opencv-python': ['opencv-python==3.4.0.12'],
    },
)
