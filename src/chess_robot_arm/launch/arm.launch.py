"""机械臂独立入口：加载模型和参数，默认只读，不自动使能或初始化。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 启动一个机械臂节点；参数按基础配置、Home、历史参考、launch 覆盖值的顺序加载。
def generate_launch_description():
    share = get_package_share_directory('chess_robot_arm')
    model = os.path.join(
        get_package_share_directory('chess_robot_description'),
        'urdf', 'nova5_with_ag95.urdf',
    )
    return LaunchDescription([
        # 回环值仅为模板默认值，实际控制器地址通过启动参数指定。
        DeclareLaunchArgument('controller_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('urdf_file', default_value=model),
        DeclareLaunchArgument(
            'safety_model_file',
            default_value=os.path.join(share, 'config', 'nova5_ag95_geometry.yaml'),
        ),
        Node(
            package='chess_robot_arm',
            executable='arm_node',
            name='chess_robot_arm',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'arm.yaml'),
                os.path.join(share, 'config', 'home.yaml'),
                os.path.join(share, 'config', 'twist_reference.yaml'),
                {
                    'urdf_file': LaunchConfiguration('urdf_file'),
                    'controller.ip': LaunchConfiguration('controller_ip'),
                    'safety.model_file': LaunchConfiguration('safety_model_file'),
                },
            ],
        ),
    ])
