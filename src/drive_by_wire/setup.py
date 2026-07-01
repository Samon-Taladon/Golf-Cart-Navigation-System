from setuptools import setup
import os
from glob import glob

package_name = 'drive_by_wire'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files if any
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='ROS2 package for drive-by-wire system using joystick input to control cmd_vel.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_by_wire_node = drive_by_wire.drive_by_wire_node:main',
            'speed_node = drive_by_wire.speed_node:main',
            'steering_node = drive_by_wire.steering_node:main',
            'brake_node = drive_by_wire.brake_node:main',
        ],
    },
)