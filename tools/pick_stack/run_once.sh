#!/usr/bin/env bash
# One finite business workflow; connects only to existing ROS nodes.
set -eo pipefail
entry_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$entry_dir/../.."
source /opt/ros/humble/setup.bash
source runs/project_release_20261102_01/install/setup.bash
export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
exec python3 tools/pick_stack/demo.py "$@"
