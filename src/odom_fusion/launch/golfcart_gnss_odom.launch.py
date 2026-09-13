"""Start low-level, GNSS, fusion, and optional RViz without changing either source package."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_low_level = LaunchConfiguration('start_low_level')
    start_gnss = LaunchConfiguration('start_gnss')
    start_rviz = LaunchConfiguration('start_rviz')
    speed_scale = LaunchConfiguration('speed_scale')

    low_level_launch = PathJoinSubstitution([
        FindPackageShare('golfcart_low_level'), 'launch', 'low_level.launch.py'
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('odom_fusion'), 'rviz', 'golfcart_odom.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_low_level', default_value='true',
            description='Start golfcart_low_level low_level.launch.py.'),
        DeclareLaunchArgument(
            'start_gnss', default_value='true',
            description='Start navigation_system enhanced_gnss_publisher.'),
        DeclareLaunchArgument(
            'start_rviz', default_value='true',
            description='Start RViz with fused odometry displays.'),
        DeclareLaunchArgument(
            'speed_scale', default_value='0.8333333333333334',
            description='Calibration multiplier for /speed_status (actual / measured).'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(low_level_launch),
            condition=IfCondition(start_low_level),
        ),
        Node(
            package='navigation_system',
            executable='enhanced_gnss_publisher',
            name='gnss_pose_publisher',
            output='screen',
            remappings=[('/odom', '/gnss/odom')],
            condition=IfCondition(start_gnss),
        ),
        Node(
            package='odom_fusion',
            executable='odom_fusion_node',
            name='odom_fusion_node',
            output='screen',
            parameters=[{'speed_scale': speed_scale}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(start_rviz),
        ),
    ])
