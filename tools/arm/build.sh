#!/usr/bin/env bash
set -euo pipefail
arm_tool_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
arm_project="$(cd -- "$arm_tool_dir/../.." && pwd)"
mkdir -p "$arm_project/build"
cp "$arm_project/docs/build-layout.txt" "$arm_project/build/README.txt"
cmake -S "$arm_tool_dir" -B "$arm_project/build/arm_tools" -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
cmake --build "$arm_project/build/arm_tools" --parallel 2
ctest --test-dir "$arm_project/build/arm_tools" --output-on-failure
