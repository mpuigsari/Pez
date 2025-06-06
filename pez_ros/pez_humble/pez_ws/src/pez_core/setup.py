from setuptools import setup, find_packages

package_name = 'pez_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        # ament index entry
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # config files (YAML and RQt perspective)
        ('share/' + package_name + '/config', [
            'config/joystick_params.yaml',
            'config/axis_params.yaml',
            'config/pez.perspective',
            'config/test.xml'
        ]),
        # launch files
        ('share/' + package_name + '/launch', [
            'launch/joy_launch.py',
            'launch/teleop_launch.py',
        ]),
        # ─── NEW: install our .srv files ───
        ('share/' + package_name + '/srv', [
            'srv/GetSensors.srv',
        ]),
    ],
    install_requires=['setuptools', 'bluerobotics-navigator'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Pez control nodes ported to ROS 2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fish_teleop = pez_core.fish_teleop:main',
            'fish_joy        = pez_core.fish_joy:main',
            'fish_sense = pez_core.fish_sense:main',
        ],
    },
)
