#!/usr/bin/env python3

from setuptools import setup, find_packages

setup(
    name='pez_comms',
    version='0.1.0',
    description='Generic serial communications for ROS Noetic via YAML-driven configuration',
    author='max',
    author_email='',
    url='',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    install_requires=[
        'pyyaml',
    ],
    entry_points={
        'console_scripts': [
            # exposes `comms_node` on your $PATH (rosrun, etc)
            'comms_node = pez_comms.nodes.comms_node:main',
        ],
    },
)
