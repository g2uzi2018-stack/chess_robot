#!/usr/bin/env bash
# Build the development workspace; does not start or restart any node.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
source /opt/ros/humble/setup.bash
mkdir -p build
cp docs/build-layout.txt build/README.txt
colcon --log-base build/colcon_logs build --base-paths src   --build-base build/ros --install-base install/dev --symlink-install   --executor sequential "$@"
