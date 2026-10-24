#!/usr/bin/env bash
set -euo pipefail
arm_tool_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
arm_project="$(cd -- "$arm_tool_dir/../.." && pwd)"
cmake -S "$arm_tool_dir" -B "$arm_project/runs/arm_tools_build" -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
cmake --build "$arm_project/runs/arm_tools_build" --parallel 2
ctest --test-dir "$arm_project/runs/arm_tools_build" --output-on-failure
