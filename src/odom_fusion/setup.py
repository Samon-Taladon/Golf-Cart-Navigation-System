from glob import glob
from setuptools import setup

package_name = 'odom_fusion'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2_ws',
    maintainer_email='maintainer@example.com',
    description='GNSS pose and measured speed fusion node.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_fusion_node = odom_fusion.odom_fusion_node:main',
        ],
    },
)
