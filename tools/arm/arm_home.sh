#!/usr/bin/env bash
set -euo pipefail
arm_tool_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
arm_project="$(cd -- "$arm_tool_dir/../.." && pwd)"
exec "$arm_project/runs/arm_tools_build/arm_operator" home --config "$arm_tool_dir/config/operator.yaml" --home-file "$arm_project/config/arm/home.yaml" "$@"
