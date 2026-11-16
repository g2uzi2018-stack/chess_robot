#!/usr/bin/env bash
set -e
cd "${CHESS_ROBOT_ROOT:-$(dirname -- "${BASH_SOURCE[0]}")/../../..}"
source tools/env.sh
export ROS_DOMAIN_ID=90
export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$PWD/install/dev/chess_robot_vision/share/chess_robot_vision/config/large_images_fastdds.xml}"
exec ros2 launch chess_robot_preview camera_preview.launch.py
