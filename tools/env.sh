# Source from Bash to use this checkout's development build.
chess_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$chess_root/install/dev/local_setup.bash"
export CHESS_ROBOT_ROOT="$chess_root"
export ROS_LOG_DIR="$chess_root/log/$(date +%F)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-90}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RCUTILS_CONSOLE_OUTPUT_FORMAT='{time} [{severity}] [{name}] {message}'
export RCUTILS_COLORIZED_OUTPUT=0
