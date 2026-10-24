"""Read-only defaults. This launch does not enable or initialize the robot."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('chess_robot_arm')
    model = os.path.join(get_package_share_directory('chess_robot_description'),
                         'urdf', 'nova5_with_ag95.urdf')
    return LaunchDescription([
        DeclareLaunchArgument('controller_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('urdf_file', default_value=model),
        Node(package='chess_robot_arm', executable='arm_node', name='chess_robot_arm',
             output='screen', parameters=[os.path.join(share, 'config', 'arm.yaml'),
                                         os.path.join(share, 'config', 'home.yaml'),
                                         {'urdf_file': LaunchConfiguration('urdf_file'),
                                          'controller.ip': LaunchConfiguration('controller_ip')}])
    ])
