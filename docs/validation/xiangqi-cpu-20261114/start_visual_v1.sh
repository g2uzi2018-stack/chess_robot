#!/usr/bin/env bash
set -eo pipefail
cd /home/kuang/workspace/chess_robot
source /opt/ros/humble/setup.bash
source /home/kuang/workspace/chess_robot/runs/project_release_20261102_01/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/kuang/workspace/chess_robot/runs/project_release_20261102_01/install/chess_robot_vision/share/chess_robot_vision/config/large_images_fastdds.xml
export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
# Preserve the existing managed fixed-clock lease lifecycle; no new power policy.
python3 /home/kuang/workspace/chess_robot/runs/vision_acceleration_20261101_01/clock_lease.py acquire
export XIANGQI_MODEL_ARTIFACT=/home/kuang/workspace/chess_robot/runs/fast_accurate_20261113_01/artifacts/fp16/roi-batch4-fp16.engine
export XIANGQI_MODEL_ARTIFACT_SHA256=57542cda1c56d237d48f20bb8848d97ff7ab15b95864ac7bb7583619255de232
export XIANGQI_SOURCE_WEIGHTS_SHA256=f4fce2c0217b2ce292820e45f1779654b4cf58da3d156aff6306d56ae6aabf7e
TRT=/home/kuang/workspace/chess_robot/runs/perf_ceiling_20261031/01_backend/deps/trt-root
DEPS=/home/kuang/workspace/chess_robot/runs/perf_ceiling_20261031/01_backend/deps/python
export PYTHONPATH="$DEPS:$TRT/usr/lib/python3.10/dist-packages:$TRT/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$TRT/usr/lib/aarch64-linux-gnu:$TRT/usr/lib/aarch64-linux-gnu/nvidia:$TRT/usr/lib/aarch64-linux-gnu/tegra:${LD_LIBRARY_PATH:-}"
export XIANGQI_RUN_DIR=/home/kuang/workspace/chess_robot/runs/row_repeat_20261109_01/vision-production
export XIANGQI_SOURCE_DIR=/home/kuang/workspace/chess_robot/runs/cpu_optimization_20261114_01/vision-source-v1
unset XIANGQI_BENCHMARK_LOCK
export XIANGQI_CANDIDATE_MODE=lite
export XIANGQI_ROI_BATCH_SIZE=4
export XIANGQI_ROI_PLANNER=grouped
export XIANGQI_ROI_GROUP_MAX_SIZE=704
# 同帧候选缺少分割掩膜时，增加一次672像素上下文补查，总ROI预算仍为8。
export XIANGQI_ROI_CONTEXT_RECHECK_SIZE=672
export XIANGQI_CANDIDATE_ARTIFACT=/home/kuang/workspace/xiangqi-yolo-training/20261102-yolo26n-piece-v1/artifacts/yolo26n-piece-fp32-v1/imgsz640-fp32/yolo26n-piece-imgsz640-fp32.engine
export XIANGQI_CANDIDATE_ARTIFACT_SHA256=5ef87b3e4f6fd3991586845687a30ee8b5a0819019eb96cd73ebac1a80953a7b
export XIANGQI_CANDIDATE_CONF=0.1
export XIANGQI_PIECE_CLASSIFIER=/home/kuang/workspace/chess_robot/runs/fast_accurate_20261113_01/artifacts/classifier-trt-v1/classifier.engine
export XIANGQI_PIECE_CLASSIFIER_SHA256=aabc945ff9bf4deec82a25fd1c353721071c7fd4d884e6dfa65d6cbedef7f0cb
export XIANGQI_SURFACE_FIT=/home/kuang/workspace/chess_robot/runs/position_stability_20261113_01/candidate-v4/surface_center.so
export XIANGQI_SURFACE_FIT_SHA256=f226c32b970a7e9a74ba44a05a1f8694b4a3edabc59f4410d97b2ee64eec0d0b
export XIANGQI_SURFACE_FIT_METHOD=center_recovery
exec /home/kuang/workspace/xiangqi-yolo-training/20261031-yolo26s-seg-v1/runtime/python-yolo "$XIANGQI_SOURCE_DIR/live_tracked.py"
