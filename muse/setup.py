from setuptools import (
    find_packages,
    setup,
)

setup(
    name='iawake.muse',
    version='0.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    dependency_links=[
        'git+https://github.com/Automa-Cognoscenti/libs.git#egg=iawake.core-0.0.0#subdirectory=core',
    ],
    install_requires=[
        'iawake.core==0.0.0',
    ],
)
