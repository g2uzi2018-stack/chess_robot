import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(get_package_share_directory('chess_robot_preview'), 'config', 'channels.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('preview_config', default_value=config),
        Node(package='chess_robot_preview', executable='preview_node', name='preview_node',
             parameters=[{'config_file': LaunchConfiguration('preview_config')}], output='screen'),
    ])
