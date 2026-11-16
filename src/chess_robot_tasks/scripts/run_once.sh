#!/usr/bin/env bash
# One finite business workflow; connects only to existing ROS nodes.
set -eo pipefail
entry_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${CHESS_ROBOT_ROOT:-$entry_dir/../../..}"
source tools/env.sh
export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
exec python3 -m chess_robot_tasks.demo "$@"
