#!/usr/bin/env bash
set -eo pipefail
cd /home/kuang/workspace/chess_robot
source /opt/ros/humble/setup.bash
source install/setup.bash
source install_vision/setup.bash
export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
export XIANGQI_RUN_DIR=/home/kuang/workspace/chess_robot/runs/xiangqi_tracking_20261031_01
exec /home/kuang/workspace/xiangqi-yolo-training/20261031-yolo26s-seg-v1/runtime/python-yolo /home/kuang/workspace/chess_robot/tools/testing/xiangqi_tracking/live_tracked.py
