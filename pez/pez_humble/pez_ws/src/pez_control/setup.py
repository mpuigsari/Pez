from setuptools import setup

package_name = 'pez_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
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
        ]),
        # launch files
        ('share/' + package_name + '/launch', [
            'launch/joy_launch.py',
            'launch/controller_launch.py',
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
            'axis_controller = pez_control.pez_axis_controller:main',
            'pez_joy        = pez_control.pez_joy:main',
            'pez_test        = pez_control.pez_test:main',
        ],
    },
)
