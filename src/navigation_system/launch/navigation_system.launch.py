from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('navigation_system')
    default_web_dir = os.path.join(pkg_dir, 'web')
    
    web_server_enabled = LaunchConfiguration('web_server')
    web_dir = LaunchConfiguration('web_dir')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'web_server',
            default_value='true',
            description='Enable the web server'
        ),
        
        DeclareLaunchArgument(
            'web_dir',
            default_value=default_web_dir,
            description='Directory containing web files'
        ),
        
        # Start rosbridge server for ROS2 web communication
        ExecuteProcess(
            cmd=["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"],
            output="screen",
            condition=IfCondition(web_server_enabled)
        ),
        
        # Start simple HTTP server for web interface
        ExecuteProcess(
            cmd=[
                FindExecutable(name='python3'),
                "-m",
                "http.server",
                "8080",
                "--directory",
                web_dir
            ],
            output="screen",
            condition=IfCondition(web_server_enabled)
        ),
        
        # เปิด GNSS publisher node
        Node(
            package="navigation_system",
            executable="gnss_publisher",
            name="gnss_publisher",
            output="screen"
        ),
        
        # เปิด bearing subscriber node
        Node(
            package="navigation_system",
            executable="bearing_subscriber",
            name="bearing_subscriber",
            output="screen"
        ),
         Node(
            package="navigation_system",
            executable="pure_pursuit_navigator",
            name="pure_pursuit_navigator",
            output="screen"
        )
    ])