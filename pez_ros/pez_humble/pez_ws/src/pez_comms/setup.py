from setuptools import setup, find_packages

package_name = 'pez_comms'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    install_requires=[
        'pyserial',
        'pyyaml',
    ],
    data_files=[
        # ament index entry
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # config folder
        ('share/' + package_name + '/config', [
            'config/fish_comms.yaml',
            'config/host_comms.yaml',
        ]),
        # launch files
        ('share/' + package_name + '/launch', [
            'launch/comms.launch.py',
            'launch/fish.launch.py',
            'launch/host.launch.py',
            'launch/test_comms.launch.py',
        ]),
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Config-driven communications node for Pez robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'comms = pez_comms.nodes.comms_node:main',
        ],
    },
)
