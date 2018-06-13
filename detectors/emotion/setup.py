from setuptools import (
    find_packages,
    setup,
)

setup(
    name='iawake.emotion',
    version='0.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    dependency_links=[
        'git+https://github.com/Automa-Cognoscenti/libs.git#egg=iawake.core-0.0.0#subdirectory=core',
    ],
    install_requires=[
        'h5py==2.7.1',
        'iawake.core==0.0.0',
        'keras==2.1.2',
        'numpy==1.14.3',
        'schematics==2.0.1',
        'tensorflow==1.8.0',
    ],
)
