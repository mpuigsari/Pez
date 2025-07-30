from setuptools import find_packages, setup

package_name = 'pez_joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # config files (YAML, RQt perspective)
        ('share/' + package_name + '/config', [
            'config/bluerov_joy_config.yaml',
            'config/pez_rqt.perspective',
            'config/pez_plot.xml',
            'config/pez_joy_config.yaml',
            'config/pez_experiment.yaml',
        ]),
        # launch files
        ('share/' + package_name + '/launch', [
            'launch/pez_launch.py',
            'launch/joy_launch.py',
            'launch/comms_launch.py',
            'launch/display_launch.py',
            'launch/experiment_launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bluerov_joy = pez_joy.bluerov_joy:main',
            'pez_joy     = pez_joy.pez_joy:main',
            'command_player = pez_joy.experiment.cmd_svc_player:main',
            'joy_player = pez_joy.experiment.joy_player:main',

        ],
    },
)
