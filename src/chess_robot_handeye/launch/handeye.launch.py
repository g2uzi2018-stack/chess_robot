from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', description='Absolute path to a reviewed handeye YAML'),
        Node(package='chess_robot_handeye', executable='handeye_node', output='screen',
             parameters=[{'config_file': LaunchConfiguration('config_file')}]),
    ])
