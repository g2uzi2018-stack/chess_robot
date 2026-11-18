from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# 启动单个夹爪节点；config 可指定其他 YAML，默认使用不连接硬件的模拟配置。
def generate_launch_description():
    # 从已安装包的 share 目录查找配置；不依赖运行命令时所在的目录。
    default = os.path.join(get_package_share_directory('chess_robot_gripper'), 'config', 'mock.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        # 这里只加载参数并启动进程，不发送初始化或开合请求。
        Node(package='chess_robot_gripper', executable='gripper_node',
             parameters=[LaunchConfiguration('config')], output='screen'),
    ])
