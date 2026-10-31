#!/usr/bin/env bash
set -eo pipefail
cd /home/kuang/workspace/chess_robot
source /opt/ros/humble/setup.bash
source runs/xiangqi_tracking_20261031_01/install/setup.bash
export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$PWD/install_vision/chess_robot_vision/share/chess_robot_vision/config/large_images_fastdds.xml}"
exec runs/xiangqi_tracking_20261031_01/install/chess_robot_tracking_preview/lib/chess_robot_tracking_preview/tracking_preview
