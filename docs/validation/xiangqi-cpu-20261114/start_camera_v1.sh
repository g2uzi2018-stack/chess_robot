#!/usr/bin/env bash
set -eo pipefail
cd /home/kuang/workspace/chess_robot
source /opt/ros/humble/setup.bash
source /home/kuang/workspace/chess_robot/runs/project_release_20261102_01/install/setup.bash
export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
export LD_LIBRARY_PATH=/home/kuang/.local/realsense-2.58.1/lib:${LD_LIBRARY_PATH:-}
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/kuang/workspace/chess_robot/runs/project_release_20261102_01/install/chess_robot_vision/share/chess_robot_vision/config/large_images_fastdds.xml
source /home/kuang/workspace/chess_robot/runs/vision_hardware_20261105_01/install/setup.bash
export GST_PLUGIN_PATH=/home/kuang/workspace/chess_robot/runs/vision_hardware_20261105_01/deps/root/usr/lib/aarch64-linux-gnu/gstreamer-1.0
export GST_REGISTRY=/home/kuang/workspace/chess_robot/runs/vision_hardware_20261105_01/deps/registry.bin
export LD_LIBRARY_PATH=/home/kuang/workspace/chess_robot/runs/vision_hardware_20261105_01/deps/root/usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu/nvidia:${LD_LIBRARY_PATH:-}
source /home/kuang/workspace/chess_robot/runs/cpu_optimization_20261114_01/camera-install-v1/local_setup.bash
exec ros2 launch chess_robot_preview camera_preview.launch.py jpeg_backend:=nvjpeg
