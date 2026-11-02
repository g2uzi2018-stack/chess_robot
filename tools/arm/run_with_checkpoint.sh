#!/usr/bin/env bash
# Reusable local installation entry point; never initializes zero winding.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
prefix="$root/install"
if [ -f "$root/.runtime/arm-install-prefix" ]; then
  IFS= read -r prefix < "$root/.runtime/arm-install-prefix"
fi
test -f "$root/.runtime/arm.yaml" || { printf '%s\n' 'Missing installation configuration: .runtime/arm.yaml' >&2; exit 1; }
python3 "$root/tools/arm/validate_home.py" --root "$root"
source /opt/ros/humble/setup.bash
source "$prefix/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-90}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
exec "$prefix/chess_robot_arm/lib/chess_robot_arm/arm_node" --ros-args \
  --params-file "$root/.runtime/arm.yaml" \
  --params-file "$root/config/arm/home.yaml" \
  --params-file "$root/config/arm/twist_reference.yaml"
