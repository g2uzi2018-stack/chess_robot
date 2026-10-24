#!/usr/bin/env bash
# Offline analysis only: no actual controller commands or ROS robot publishers.
set -euo pipefail
arm_analysis_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
arm_project="$(cd -- "$arm_analysis_dir/../.." && pwd)"
arm_output="${1:-$arm_project/runs/workspace_$(date -u +%Y%m%dT%H%M%S)}"
if (($#)); then shift; fi
arm_python="${ARM_ANALYSIS_PYTHON:-$arm_project/runs/urdf_tools_env/bin/python}"
if [[ ! -x "$arm_python" ]] || [[ ! -f "$arm_project/install/chess_robot_arm/lib/libchess_robot_arm_core.a" && ! -f "$arm_project/install/chess_robot_arm/lib/libchess_robot_arm_core.so" ]]; then
  echo 'Prepare the analysis Python environment and build chess_robot_arm first; see tools/analysis/README.md.' >&2
  exit 2
fi
mkdir -p "$arm_output"
arm_output="$(cd -- "$arm_output" && pwd)"
"$arm_python" "$arm_analysis_dir/workspace_analysis.py" --project "$arm_project" --out "$arm_output" "$@"
set +u
source /opt/ros/humble/setup.bash
source "$arm_project/install/setup.bash"
set -u
g++ -O2 -std=c++17 -I"$arm_project/install/chess_robot_arm/include" -I"$arm_project/install/chess_robot_twist/include" -I/usr/include/eigen3 \
  "$arm_analysis_dir/workspace_policy_probe.cpp" -L"$arm_project/install/chess_robot_arm/lib" -L"$arm_project/install/chess_robot_twist/lib" \
  -lchess_robot_arm_core -lchess_robot_twist -o "$arm_project/runs/workspace_policy_probe"
"$arm_python" "$arm_analysis_dir/workspace_policy_cases.py" --project "$arm_project" --baseline "$arm_output" --out "$arm_output"
"$arm_output/workspace_eval" "$arm_output/model.yaml" "$arm_output/policy_cases.bin" "$arm_output/policy_case_metrics.bin"
"$arm_python" "$arm_analysis_dir/workspace_extra_stats.py" "$arm_output"
"$arm_python" "$arm_analysis_dir/workspace_sweeps.py" --project "$arm_project" --run "$arm_output"
"$arm_python" "$arm_analysis_dir/workspace_selfcheck.py" --project "$arm_project" --out "$arm_output"
"$arm_python" "$arm_analysis_dir/workspace_verify.py" --project "$arm_project" --run "$arm_output"
"$arm_python" "$arm_analysis_dir/workspace_plot.py" --project "$arm_project" --run "$arm_output" --out "$arm_output/figures"
echo "Offline study complete: $arm_output/figures"
