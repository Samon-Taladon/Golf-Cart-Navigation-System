import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('navigation_system')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'controller.rviz')
    rviz_enabled = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz2 with the controller visualization config',
        ),
        Node(
            package='navigation_system',
            executable='enhanced_gnss_publisher',
            name='enhanced_gnss_publisher',
            output='screen',
        ),
        Node(
            package='navigation_system',
            executable='imu',
            name='imu',
            output='screen',
        ),
        Node(
            package='navigation_system',
            executable='purepursuit',
            name='purepursuit',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(rviz_enabled),
        ),
    ])
