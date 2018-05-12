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
        'git+ssh://git@github.com:Automa-Cognoscenti/libs.git'
    ],
    install_requires=[
        'iawake.core==0.0.0',
        'keras==2.1.6',
        'numpy==1.14.3',
        'tensorflow==1.8.0',
    ],
)
