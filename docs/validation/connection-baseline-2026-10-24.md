# C++ connection baseline — 2026-10-24

Source requirement: [中国象棋夹取](https://app.notion.com/p/3ff9f9a4e09e81fca280e1f3e4c42140).

This increment strengthens the existing read-only transport and creates a
repeatable offline test entry point before implementing more controller logic.
It does not establish controller identity, firmware, protocol compatibility,
actual feedback correctness or motion acceptance.

## Repository and changes

- Development checkout: `/home/kuang/workspace/chess_robot` on AGX.
- Branch: `feat/ag95-description-20261023`; base HEAD:
  `4dbd56ad5836216e496c6bb3b78f057fa4e2e46f`.
- Initial working tree was clean. This increment is left as uncommitted changes
  for review; it has not been pushed.
- `net.hpp`: reject invalid ports and nonpositive timeouts; check nonblocking
  setup; handle interrupted polling; retain buffered bytes at hangup; distinguish
  EOF and system errors; share a monotonic deadline across query send and reply.
- Query default is now 3000 ms total including sending (previously separate
  send/reply budgets), with a 500 ms per-byte wait cap and a 16384-byte reply cap
  including the semicolon. Existing command allowlists are unchanged.
- `transport_check.cpp`: loopback tests for argument validation, fragmented
  delivery, truncated EOF, silence, continuous slow data, query replies and
  exact/oversized response boundaries.
- CMake: register `readonly_allowlist`, `feedback_frames`, `transport_loopback`,
  each with a 20 s test-runner timeout. README documents build and test commands.

## Validation and evidence

Evidence directory: `runs/20261024_connection_baseline/` (ignored by Git).

- AGX native build: success. Initial unrestricted colcon discovery also scanned
  the existing Python environment under `runs/urdf_tools_env` and emitted package
  identification errors unrelated to the C++ build. Final commands restrict
  discovery using `--base-paths src`; no environment was removed or modified.
- Registered suite: **3 tests, 0 failures, 0 errors, 0 skipped**. These are offline
  and loopback tests, not three hardware acceptance results.
- `git diff --check`: passed.

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select chess_robot_validation --cmake-args -DBUILD_TESTING=ON
colcon test --base-paths src --packages-select chess_robot_validation --event-handlers console_direct+
colcon test-result --test-result-base build/chess_robot_validation --verbose
```

## Current endpoint observation

TCP connect/close only to the project-recorded `192.168.30.234` from AGX:
29999 and 30004 accepted connections; 30003 refused. No application command
bytes were sent. This observation establishes endpoint reachability at capture
time, not a Nova5 identity or a V3/V4 protocol conclusion. No feedback frames
were captured and no ROS feedback publication was started.

Before the next hardware integration increment, record the controller identity,
firmware version and matching TCP/IP protocol, then validate real feedback
against the actual configuration. TCP/tool calibration and the unresolved
physical/model collision discrepancy remain outstanding project requirements.
