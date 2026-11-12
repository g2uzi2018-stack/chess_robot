#!/usr/bin/env bash
set -eo pipefail
HERE=/home/kuang/workspace/chess_robot/runs/vision_acceleration_20261101_01
TRT=/home/kuang/workspace/chess_robot/runs/perf_ceiling_20261031/01_backend/deps/trt-root
DEPS=/home/kuang/workspace/chess_robot/runs/perf_ceiling_20261031/01_backend/deps/python
export PYTHONPATH="$DEPS:$TRT/usr/lib/python3.10/dist-packages:$TRT/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$TRT/usr/lib/aarch64-linux-gnu:$TRT/usr/lib/aarch64-linux-gnu/nvidia:$TRT/usr/lib/aarch64-linux-gnu/tegra:${LD_LIBRARY_PATH:-}"
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
exec /home/kuang/workspace/xiangqi-yolo-training/20261031-yolo26s-seg-v1/runtime/python-yolo "$@"
