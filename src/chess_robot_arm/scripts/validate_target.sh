#!/usr/bin/env bash
# Run with bash from the repository root on ROS 2 Humble AGX; offline tests only.
set -euo pipefail
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'BLOCKED: ROS 2 Humble is unavailable; no build/test was performed.' >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
set -u
command -v c++ >/dev/null || { echo 'BLOCKED: C++ compiler unavailable' >&2; exit 2; }
command -v colcon >/dev/null || { echo 'BLOCKED: colcon unavailable' >&2; exit 2; }
colcon build --base-paths src --packages-select chess_robot_twist chess_robot_interfaces chess_robot_arm --cmake-args -DBUILD_TESTING=ON
set +u
source install/setup.bash
set -u
colcon test --base-paths src --packages-select chess_robot_twist chess_robot_arm --event-handlers console_direct+
colcon test-result --test-result-base build/chess_robot_twist/test_results --verbose
colcon test-result --test-result-base build/chess_robot_arm/test_results --verbose
