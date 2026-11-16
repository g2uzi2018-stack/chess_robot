#!/usr/bin/env bash
set -eo pipefail
vision_domain="${ROS_DOMAIN_ID:-90}"
vision_localhost="${ROS_LOCALHOST_ONLY:-1}"
cd "${CHESS_ROBOT_ROOT:-$(dirname -- "${BASH_SOURCE[0]}")/../../..}"
source /opt/ros/humble/setup.bash
prefix="${XIANGQI_RENDER_PREFIX:-$PWD/install/dev}"
source "$prefix/local_setup.bash"
export ROS_DOMAIN_ID="$vision_domain" ROS_LOCALHOST_ONLY="$vision_localhost" OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$PWD/install/dev/chess_robot_vision/share/chess_robot_vision/config/large_images_fastdds.xml}"
exec "$prefix/chess_robot_tracking_preview/lib/chess_robot_tracking_preview/tracking_preview" "$@"
