#!/usr/bin/env bash
set -eo pipefail
vision_domain="${ROS_DOMAIN_ID:-90}"
vision_localhost="${ROS_LOCALHOST_ONLY:-1}"
cd "${CHESS_ROBOT_ROOT:-$(dirname -- "${BASH_SOURCE[0]}")/../../..}"
export CHESS_ROBOT_ROOT="$PWD"
source tools/env.sh
export ROS_DOMAIN_ID="$vision_domain" ROS_LOCALHOST_ONLY="$vision_localhost" OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
export XIANGQI_RUN_DIR="${XIANGQI_RUN_DIR:-$PWD/runs/vision_$(date +%Y%m%d_%H%M%S_%N)}"
exec "${XIANGQI_PYTHON:-/usr/bin/python3}" -m chess_robot_perception.scene_node "$@"
