from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    default = os.path.join(get_package_share_directory('chess_robot_vision'), 'config', 'd405.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        Node(package='chess_robot_vision', executable='vision_node', name='vision_node',
             parameters=[LaunchConfiguration('config')], output='screen'),
    ])
