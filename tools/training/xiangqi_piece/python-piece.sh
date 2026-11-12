#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/kuang/workspace/xiangqi-yolo-training/20261102-yolo26n-piece-v1
OLD=/home/kuang/workspace/xiangqi-yolo-training/20261031-yolo26s-seg-v1
BACKEND=/home/kuang/workspace/chess_robot/runs/perf_ceiling_20261031/01_backend
TRT=$BACKEND/deps/trt-root
export PYTHONPATH="$BACKEND/deps/python:$TRT/usr/lib/python3.10/dist-packages:$TRT/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/home/kuang/workspace/xiangqi-labeler/.venv-sam-cuda/lib/labeler-native:$TRT/usr/lib/aarch64-linux-gnu:$TRT/usr/lib/aarch64-linux-gnu/nvidia:$TRT/usr/lib/aarch64-linux-gnu/tegra:/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu/nvidia:${LD_LIBRARY_PATH:-}"
export YOLO_CONFIG_DIR="$ROOT/runtime/ultralytics-config"
export MPLCONFIGDIR="$ROOT/runtime/matplotlib"
export XDG_CACHE_HOME="$ROOT/runtime/cache"
export TORCH_HOME="$ROOT/weights/torch-home"
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 YOLO_AUTOINSTALL=false
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=4
unset HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy
exec "$OLD/.venv/bin/python" "$@"
