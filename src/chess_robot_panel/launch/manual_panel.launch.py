"""Start panel + one gripper driver, defaulting to simulation. No arm/camera nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('panel_config', default_value=os.path.join(
            get_package_share_directory('chess_robot_panel'), 'config', 'panel.yaml')),
        DeclareLaunchArgument('gripper_config', default_value=os.path.join(
            get_package_share_directory('chess_robot_gripper'), 'config', 'mock.yaml')),
        Node(package='chess_robot_gripper', executable='gripper_node',
             parameters=[LaunchConfiguration('gripper_config')], output='screen'),
        Node(package='chess_robot_panel', executable='panel_node',
             parameters=[LaunchConfiguration('panel_config')], output='screen'),
    ])
