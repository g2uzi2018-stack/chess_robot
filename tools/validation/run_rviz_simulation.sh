#!/usr/bin/env bash
# RViz-only launcher. No controller socket, robot enable, motion API or dynamics.
set -eo pipefail
task_repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$task_repo"
source /opt/ros/humble/setup.bash
source install/setup.bash
if test -z "${DISPLAY:-}"; then
  echo 'DISPLAY is missing. Run in an authorized desktop, or start a private authenticated virtual X display as documented.' >&2
  exit 2
fi
export ROS_DOMAIN_ID=62 ROS_LOCALHOST_ONLY=1
run=${1:-runs/rviz_sim_$(date +%Y%m%d_%H%M%S)}
test ! -e "$run/synthetic_motion.csv" || { echo 'Refusing to overwrite an existing recording' >&2; exit 2; }
mkdir -p "$run"
python3 - "$run/rsp.yaml" <<'PY'
from pathlib import Path
import sys,yaml
Path(sys.argv[1]).write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':Path('src/chess_robot_description/urdf/nova5.urdf').read_text(),'publish_frequency':50.0}}}))
PY
rpid= vpid=
trap 'test -z "$vpid" || kill "$vpid" 2>/dev/null || true; test -z "$rpid" || kill "$rpid" 2>/dev/null || true; wait 2>/dev/null || true' EXIT
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$run/rsp.yaml" -r joint_states:=/validation/synthetic_joint_states > "$run/robot_state_publisher.txt" 2>&1 &
rpid=$!
rviz2 -d src/chess_robot_validation/config/nova5.rviz > "$run/rviz.txt" 2>&1 &
vpid=$!
ros2 run chess_robot_validation nova5_rviz_demo src/chess_robot_description/urdf/nova5.urdf "$run/synthetic_motion.csv" | tee "$run/demo_result.txt"
python3 tools/validation/analyze_rviz.py "$run" > "$run/analysis.txt"
echo "Synthetic demo finished. Evidence: $run. Close RViz to exit the launcher."
wait "$vpid"
