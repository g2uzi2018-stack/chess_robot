#!/usr/bin/env bash
# Offline only: no production services, controller connections or camera ownership.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
baseline="$(realpath "${1:?Usage: run_history_vision_regression.sh BEFORE_SOURCE_TGZ [OUTPUT_DIR]}")"
run="${2:-$root/runs/history_vision_regression_$(date +%Y%m%d_%H%M%S)}"
test -f "$baseline"
test ! -e "$run" || { echo 'Output already exists; choose a new directory.' >&2; exit 1; }
mkdir -p "$run/source/src"
exec 9>"$root/runs/perf_ceiling_20261031/benchmark.lock"
flock 9
trap 'echo "Evidence: $run"' EXIT
for package in chess_robot_arm chess_robot_twist chess_robot_description chess_robot_interfaces chess_robot_tracking_preview; do
  cp -a "$root/src/$package" "$run/source/src/"
done
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=118 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 CHESS_ROBOT_ROOT="$root"
unset XIANGQI_BENCHMARK_LOCK
cd "$run"
colcon --log-base log build --base-paths source/src --build-base build --install-base install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
for pair in chess_robot_arm/test_arm chess_robot_arm/test_checkpoint chess_robot_twist/test_twist; do
  name=${pair##*/}
  build/$pair --gtest_output=xml:"$run/$name.xml" > "$run/$name.log" 2>&1
done
python3 "$root/tools/testing/xiangqi_tracking/test_history_loopback.py" --run "$run"
/home/kuang/workspace/xiangqi-yolo-training/20261031-yolo26s-seg-v1/runtime/python-yolo "$root/tools/testing/xiangqi_tracking/test_decoupling.py" "$baseline" > "$run/vision-tests.log" 2>&1
python3 "$root/src/chess_robot_tracking_preview/test/test_tracking_overlay.py" --binary "$run/install/chess_robot_tracking_preview/lib/chess_robot_tracking_preview/tracking_preview" --output "$run/renderer-test"
source "$root/install_vision/setup.bash"
/home/kuang/workspace/xiangqi-yolo-training/20261031-yolo26s-seg-v1/runtime/python-yolo "$root/tools/testing/xiangqi_tracking/test_image_admission.py" --output "$run/image-admission"
echo 'Offline regression passed.'
