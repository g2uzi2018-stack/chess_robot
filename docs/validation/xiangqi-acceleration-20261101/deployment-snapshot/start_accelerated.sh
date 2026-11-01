#!/usr/bin/env bash
set -eo pipefail
HERE=/home/kuang/workspace/chess_robot/runs/vision_acceleration_20261101_01
python3 "$HERE/clock_lease.py" acquire
export XIANGQI_MODEL_ARTIFACT=/home/kuang/workspace/chess_robot/runs/perf_ceiling_20261031/01_backend/weights/20261031T143600930755709/best.engine
export XIANGQI_MODEL_ARTIFACT_SHA256=26103821bc0788d1dc0a2d0051c91362530d5b6b1cc92347f2289d12c9469d35
export XIANGQI_SOURCE_WEIGHTS_SHA256=f4fce2c0217b2ce292820e45f1779654b4cf58da3d156aff6306d56ae6aabf7e
TRT=/home/kuang/workspace/chess_robot/runs/perf_ceiling_20261031/01_backend/deps/trt-root
DEPS=/home/kuang/workspace/chess_robot/runs/perf_ceiling_20261031/01_backend/deps/python
export PYTHONPATH="$DEPS:$TRT/usr/lib/python3.10/dist-packages:$TRT/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$TRT/usr/lib/aarch64-linux-gnu:$TRT/usr/lib/aarch64-linux-gnu/nvidia:$TRT/usr/lib/aarch64-linux-gnu/tegra:${LD_LIBRARY_PATH:-}"
export XIANGQI_RUN_DIR="$HERE/production"
export XIANGQI_SOURCE_DIR="$HERE/source"
export XIANGQI_BENCHMARK_LOCK=/home/kuang/workspace/chess_robot/runs/perf_ceiling_20261031/benchmark.lock
exec bash "$HERE/source/start_live_tracked.sh"
