from setuptools import setup, find_packages

package_name = 'pez_comms'

setup(
    name=package_name,
    version='0.1.0',
    # Tell setuptools to look under src/pez_comms for the Python package:
    package_dir={'': 'src'},
    packages=find_packages(where='src'),  
    install_requires=[
        'pyserial',
        'pyyaml',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', [ 'resource/pez_comms' ]),
        ('share/pez_comms', [ 'package.xml' ]),
        ('share/pez_comms/config', [
            'config/fish_comms.yaml',
            'config/host_comms.yaml',
        ]),
        ('share/pez_comms/launch', [
            'launch/comms_launch.py',
            'launch/fish_launch.py',
            'launch/host_launch.py',
            'launch/test_launch.py',
        ]),
    ],
    entry_points={
        'console_scripts': [
            'comms = pez_comms.nodes.comms_node:main',
        ],
    },
)
