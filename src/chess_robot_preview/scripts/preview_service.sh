#!/usr/bin/env bash
set -e
cd /home/kuang/workspace/chess_robot
source /opt/ros/humble/setup.bash
source install_vision/setup.bash
export ROS_DOMAIN_ID=90
export ROS_LOCALHOST_ONLY=1
exec ros2 launch chess_robot_preview camera_preview.launch.py
