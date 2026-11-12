import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    def include(package, file):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', file)))
    return LaunchDescription([
        include('chess_robot_vision', 'vision.launch.py'),
        include('chess_robot_preview', 'preview.launch.py'),
    ])
