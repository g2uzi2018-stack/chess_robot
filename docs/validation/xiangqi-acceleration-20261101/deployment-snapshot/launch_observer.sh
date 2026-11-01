#!/usr/bin/env bash
set -eo pipefail
cd /home/kuang/workspace/chess_robot
source /opt/ros/humble/setup.bash
source install/setup.bash
source install_vision/setup.bash
export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
exec python3 /home/kuang/workspace/chess_robot/runs/vision_acceleration_20261101_01/observe.py "$@"
