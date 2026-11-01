#!/usr/bin/env bash
set -eo pipefail
vision_domain="${ROS_DOMAIN_ID:-90}"
vision_localhost="${ROS_LOCALHOST_ONLY:-1}"
cd /home/kuang/workspace/chess_robot
source /opt/ros/humble/setup.bash
source install/setup.bash
source install_vision/setup.bash
export ROS_DOMAIN_ID="$vision_domain" ROS_LOCALHOST_ONLY="$vision_localhost" OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
export XIANGQI_RUN_DIR="${XIANGQI_RUN_DIR:-/home/kuang/workspace/chess_robot/runs/xiangqi_tracking_20261031_01}"
exec /usr/bin/python3 "${XIANGQI_SOURCE_DIR:-/home/kuang/workspace/chess_robot/tools/testing/xiangqi_tracking}/scene_node.py" "$@"
