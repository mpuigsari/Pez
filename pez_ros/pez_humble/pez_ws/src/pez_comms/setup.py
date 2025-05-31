# pez_ws/src/pez_comms/setup.py

import os
from glob import glob
from setuptools import setup

package_name = 'pez_comms'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'pez_comms.core'],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Wireless acoustic comms (pez_comms) for ROS 2 Humble',
    license='Apache License 2.0',
    tests_require=['pytest'],

    # ========== ENTRY POINTS ==========
    # These console_scripts correspond to your two executables:
    entry_points={
        'console_scripts': [
            'host_side = pez_comms.host_side:main',
            'fish_side = pez_comms.fish_side:main',
        ],
    },

    # ========== DATA FILES ==========
    # - resource index for ament
    # - package.xml for r-o-s indexing
    # - launch files into share/<package>/launch
    data_files=[
        # Allow ament to index this package
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        # Install package.xml so ROS 2 can find the manifest
        ('share/' + package_name, ['package.xml']),
        # Install all *.py launch files into share/pez_comms/launch
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
)
