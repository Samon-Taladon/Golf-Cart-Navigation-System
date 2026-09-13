import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory('navigation_system')
    default_web_dir = os.path.join(pkg_dir, 'web')

    web_server_enabled = LaunchConfiguration('web_server')
    web_dir = LaunchConfiguration('web_dir')
    websocket_port = LaunchConfiguration('websocket_port')
    low_level_enabled = LaunchConfiguration('low_level')
    low_level_config_file = LaunchConfiguration('low_level_config_file')
    rviz_enabled = LaunchConfiguration('rviz')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'controller.rviz')
    default_low_level_config = PathJoinSubstitution([
        FindPackageShare('golfcart_low_level'),
        'config',
        'golfcart_low_level.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'web_server',
            default_value='true',
            description='Serve the zoo web page on http://localhost:8080',
        ),
        DeclareLaunchArgument(
            'web_dir',
            default_value=default_web_dir,
            description='Directory containing mapzoo_ublox_forweb.html',
        ),
        DeclareLaunchArgument(
            'websocket_port',
            default_value='5055',
            description='WebSocket port used by the zoo web page',
        ),
        DeclareLaunchArgument(
            'low_level',
            default_value='true',
            description='Launch golfcart_low_level low_level.launch.py',
        ),
        DeclareLaunchArgument(
            'low_level_config_file',
            default_value=default_low_level_config,
            description='YAML config passed to golfcart_low_level low_level.launch.py',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Open RViz2 with the controller visualization config',
        ),

        ExecuteProcess(
            cmd=[
                FindExecutable(name='python3'),
                '-m',
                'http.server',
                '8080',
                '--directory',
                web_dir,
            ],
            output='screen',
            condition=IfCondition(web_server_enabled),
        ),

        Node(
            package='navigation_system',
            executable='web_navigation_bridge_forweb',
            name='web_navigation_bridge_forweb',
            output='screen',
            parameters=[{'websocket_port': websocket_port}],
        ),
        Node(
            package='navigation_system',
            executable='enhanced_gnss_publisher_forweb',
            name='enhanced_gnss_publisher_forweb',
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
            executable='purepursuit_forweb',
            name='purepursuit_forweb',
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('golfcart_low_level'),
                    'launch',
                    'low_level.launch.py',
                ])
            ),
            launch_arguments={
                'config_file': low_level_config_file,
            }.items(),
            condition=IfCondition(low_level_enabled),
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
