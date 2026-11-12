# Portability release verification — 2026-10-30

PASS on Ubuntu 22.04 / aarch64 / ROS 2 Humble / OpenCV 4.5.4. This release was
built into a separate ignored prefix. No live service or hardware installation
was replaced and no physical motion was issued for this release task.

| Check | Result |
|---|---|
| Arm unit tests | 98 passed |
| Handeye unit tests | 18 passed, including 4 configuration-path tests |
| Synthetic integration | 14/14 passed, sequential loopback domain 119 |
| External installed-library consumer | Configured, compiled, linked and ran successfully |
| Frozen original history | 16 files / 1,817,739 records replayed successfully; pre/post SHA-256 unchanged |
| Metadata | Package XML, configuration YAML and Python syntax valid |
| Git whitespace check | Passed |

Colcon reports 132 test records (including CTest/gtest wrapper records), zero
errors/failures/skips; this is not 132 distinct physical experiments. Integration
cases cover successful sampling/solving, plan-only operation, rejection, false
arrival, stale input, absent/unhealthy/mismatched vision, cancellation and drift,
duplicate/backward timestamps, invalid and unpaired stop feedback. The fixtures
also verify original/resolved config snapshots and model resource resolution.

The path tests cover YAML-relative paths independent of process working directory,
unchanged absolute paths, package resource lookup and malformed/unknown package
URIs. The external consumer uses installed headers and
`chess_robot_handeye::handeye_core`, rather than build-tree include paths.

Initial configuration exposed a CMake plain/keyword link signature conflict in
the newly added dependency; it was corrected to PUBLIC dependency syntax before
the successful complete build and tests. No quality/motion guard was weakened to
make a test pass. Existing arm and camera/preview behavior is retained except for
the explicitly documented prior recovery changes and 1280 configuration.

New handeye node SHA-256:
`db65cd2213dd96fa25c3b5d3f531784d79be1dc39bfda34f5c8c70fc9e71a0e5`.
Machine-readable source hashes, case outcomes and replay result are in
[release-verification.json](release-verification.json) and
[history-replay.json](history-replay.json). Historical physical calibration used
the different binary identified in the parent field report. This new binary has
not undergone another physical campaign and was not deployed by this task.

Build/test locations in the original workspace:

```
runs/handeye_release_20261030/build
runs/handeye_release_20261030/install
runs/handeye_release_20261030/consumer
runs/release-build-20261030.log
```

Reproduce after building/sourcing existing dependencies (see package README):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source install_vision/setup.bash
colcon --log-base runs/handeye_release_recheck/log build \
  --base-paths src/chess_robot_arm src/chess_robot_handeye \
  --build-base runs/handeye_release_recheck/build \
  --install-base runs/handeye_release_recheck/install \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source runs/handeye_release_recheck/install/setup.bash
colcon --log-base runs/handeye_release_recheck/test_log test \
  --base-paths src/chess_robot_arm src/chess_robot_handeye \
  --build-base runs/handeye_release_recheck/build \
  --install-base runs/handeye_release_recheck/install \
  --executor sequential --return-code-on-test-failure
colcon test-result --test-result-base runs/handeye_release_recheck/build --verbose
cmake -S src/chess_robot_handeye/test/consumer -B runs/handeye_release_recheck/consumer
cmake --build runs/handeye_release_recheck/consumer
./runs/handeye_release_recheck/consumer/consumer
```

Frozen-history replay requires the exact historical raw files and explicit
boundaries; the repository does not distribute that full dataset. The offline
CLI and synthetic regressions remain available without it. The replay's supplied
endpoint is an in-memory test snapshot, not a fresh observation of the robot.
