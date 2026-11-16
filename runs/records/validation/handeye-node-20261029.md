# Hand-eye orchestrator implementation acceptance (2026-10-29)

Historical implementation-stage record. Current field results and release scope:
[2026-10-30 report](handeye-20261030.md). Pending/failed statuses below refer to
the named earlier revisions, not the later field campaign or packaging release.

Status update: independent first-version acceptance **FAILED** on R1/R2/R3.
The targeted repair below has passed implementation regression tests and awaits
independent incremental review. Earlier test tables are retained as historical
first-version evidence, not as a current independent acceptance claim.

## Scope and result

Added `src/chess_robot_handeye`, a ROS 2 Humble C++17 stopped-pose hand-eye
orchestrator. It reads a finite YAML list, verifies existing arm parameters,
converts configured final TCP poses to PREGRASP inputs, calls the existing Action,
requires measured arrival and continuous stationary feedback, captures matched
ChArUco images/metadata and actual-joint FK flange poses, then solves only after
all points and evaluates an independent held-out subset.

The existing arm, camera, preview, messages, public interfaces and safety parameters
were not modified. No real robot motion, enable, reference initialization,
controller command, camera USB access or service restart occurred in this work.
The original two untracked analysis scripts remain untouched. No commit/push made.

## Build and tests on AGX

Environment: Ubuntu 22.04 / ROS 2 Humble / aarch64; OpenCV 4.5.4, yaml-cpp 0.7.0.

```bash
cd /home/kuang/workspace/chess_robot
source /opt/ros/humble/setup.bash
source install/setup.bash
source install_vision/setup.bash
colcon --log-base log_handeye build --base-paths src/chess_robot_handeye \
  --build-base build_handeye --install-base install_handeye \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install_handeye/setup.bash
colcon --log-base log_handeye test --base-paths src/chess_robot_handeye \
  --build-base build_handeye --install-base install_handeye --return-code-on-test-failure
colcon test-result --test-result-base build_handeye --verbose
```

Final build PASS with empty stderr (no new compiler warnings). 14 gtest cases
PASS; colcon reports 15 test records, 0 errors/failures/skips. Coverage includes
known true transform recovery (<1e-6 m/rad assertions), held-out corruption,
translation-only and parallel-axis degeneracy, missing validation, nonfinite
samples, continuous dwell/duplicate/gap resets, generated-board detection, blank
image rejection, unsupported layout, invalid quaternion and fixture/execute gates.

Eight synthetic integration cases PASS, run serially through
`test/integration.py`, with separate ROS domain 119 and `/simulation/` endpoints:

| Case | Goals | Observed outcome |
|---|---:|---|
| success | 12 | 8 train + 4 validation, PNG/metadata recorded, solve completed |
| plan | 12 | PLAN_READY only, no image samples and no accepted calibration |
| reject | 1 | Goal rejected, no retry or later point |
| no_motion | 1 | False arrival rejected against configured actual TCP |
| stale | 0 | Expired arm feedback rejected before motion request |
| camera_unhealthy | 1 | stream_ok=false rejected, no later point |
| mismatch | 1 | Image/CameraInfo timestamp mismatch times out, no later point |
| cancel | 1 | Action timeout requests one cancel; fresh stationary dwell observed |

The synthetic peer independently checks every outgoing target, including the
clearance subtraction. It generates raster ChArUco images, rather than bypassing
detection. Synthetic image tests use deliberately looser 20 mm / 0.1 rad
consistency thresholds to exercise the orchestration with small virtual rotations
and raster/PnP noise. Their acceptance is NOT evidence of industrial/physical
accuracy. The shipped real-config defaults are 5 mm / 0.03 rad consistency gates,
also provisional and separate from physical task accuracy.

## Evidence

* Build/test logs: `log_handeye/`; gtest XML under
  `build_handeye/chess_robot_handeye/test_results/`.
* Final synthetic runs: `runs/handeye_integration_final_01_{case}/`.
  Each contains fixture config/log, node log, result YAML and session artifacts.
* Manifest with SHA-256 for all 11 package files and case summaries:
  `runs/handeye_delivery_20261029_manifest.json`.
  The core/node/header fingerprint exactly matches the final tested binary.
* Synthetic success consistency maxima: train 0.006724 m / 0.029068 rad;
  held-out 0.004459 m / 0.012435 rad. These are fixture diagnostics only.
* Root `.gitignore` only adds `/build_handeye/`, `/install_handeye/`,
  `/log_handeye/` to keep local artifacts out of source delivery.

## Operational limits and next acceptance

The template is disabled, contains unknown board parameters and an empty pose
list. No physical sampling poses are certified by this delivery. Verify the board
dimensions, full dictionary/ID layout and legacy pattern, actual camera view,
installation, FK model agreement and every motion/return path before execute.
The current arm downward constraint (~0.15 rad) can reject the larger tilts needed
for useful calibration; it is intentionally unchanged. PREGRASP target conversion
can also be rejected by the existing arm workspace and is not bypassed.

This release explicitly supports OpenCV 4.5 legacy ChArUco only; CMake rejects
OpenCV >=4.6, and `legacy_pattern=false` is refused. Porting to another pattern
or library version requires explicit compatibility work and validation.

Sampling uses original host receipt timestamps and stationary windows, not a
certified exposure/robot clock synchronization. It saves PNG, human-readable
sample data, raw serialized ArmState/JointState/CameraInfo/FrameInfo, stationary
proof CSV, copied configuration/URDF and source fingerprint. Every failure ends
the run; there is no automatic retry, Home return, failed-trajectory replay or
production TF publication. Cancel ACK is not physical stop; reports distinguish
observed stop from unconfirmed/late acceptance.

`accepted=true` means held-out fixed-board consistency passed. Physical point
accuracy remains unverified and requires separate measurement before grasping.
Independent reviewer acceptance is the next step; the implementation agent does
not count its own tests as the requested independent review.

## Targeted repair after independent rejection

The independent report and all its evidence under
`runs/handeye_independent_acceptance_20261029/` were read but not modified.

* R1: cancellation stop confirmation now consumes advancing original joint
  samples, pairs them with fresh ArmState, checks position differences as well as
  reported velocity, timestamps and gaps, and consumes callback anomaly
  generations. Any anomaly resets the window and excludes older buffered
  evidence; a complete new stationary dwell is required. `stop_proof.csv`, reset
  counts and explicit failure reasons are saved. Cancel ACK alone cannot set
  `physical_stop_confirmed`.
* R2: every execute goal now has a fresh exact-header visual-triplet preflight,
  sharing capture's frame/stream/encoding/dimensions/data/intrinsics validation.
  It does not require initial board detection or certify future target visibility.
  No/unhealthy/mismatched vision now produces zero execution goals. Plan mode is
  deliberately independent of visual data.
* R3: fixture configuration, statistics and ready marker use atomic file
  replacement. The harness waits for explicit complete initialization instead of
  reading a truncated file or sleeping a guessed startup duration, distinguishes
  fixture-init failures, and verifies clean process teardown before PASS.
* Validate wording now says no arm/vision IO and zero motion commands; ordinary
  rclcpp Node parameter/rosout interfaces still exist.

Repair build on AGX PASS with empty compiler stderr. The algorithm/core was not
changed; its first-version 14-case mathematical unit evidence remains applicable.
The affected orchestration/fixture paths were actually tested with the following
14 synthetic cases, all PASS in fresh directories:

| Case | Goals | Expected and observed |
|---|---:|---|
| cancel_moving_zero_velocity | 1 | continuous ~0.25 rad/s drift, reported velocity 0: stop=false |
| cancel_duplicate_stamp | 1 | repeated stamps throughout observation: stop=false |
| cancel_backward_stamp | 1 | backward stamps throughout observation: stop=false |
| cancel_invalid_feedback | 1 | invalid-reference pulses: stop=false |
| cancel_unpaired_feedback | 1 | arm/joint time mismatch beyond tolerance: stop=false |
| no_vision | 0 | preflight rejects missing visual messages |
| camera_unhealthy | 0 | preflight rejects stream_ok=false |
| mismatch | 0 | preflight rejects unmatched visual stamps |
| cancel | 1 | ordinary fresh stationary feedback: stop=true |
| plan | 12 | no vision published, plan-only completes without capture |
| reject | 1 | rejection stops without retry |
| no_motion | 1 | false arrival rejected |
| stale | 0 | stale arm input rejected |
| success | 12 | 8 train + 4 held-out, capture/solve completes |

Every cancel case has exactly one cancellation request, no retry and no later
point. The drift regression also verifies that injected positions really changed
while the proof recorded zero reported velocity. Every test verifies clean fixture
teardown. No real hardware or existing services were used.

Current repair evidence is `runs/handeye_targeted_repair_20261029/CASE/`; final
source/binary SHA-256 and the tested-binary source fingerprint are in its
`manifest.json`. The original delivery manifest and independent report remain
historical. Reproduction uses the README integration command with `--case CASE`
and a NEW output directory. The default validate case was also checked with a new
output and records `configuration_only_no_arm_or_vision_io`.

The physical and board/pose limitations above remain unchanged. A repaired
implementation's regression PASS is not the requested independent review and
does not establish physical calibration or industrial accuracy.

## Later evidence (2026-10-30)

This page preserves the implementation-stage checks above. Subsequent 48-capture
physical calibration, independent field review, qualitative corner checks and the
portability release are recorded in [the dated field report](handeye-20261030.md).
The new path/install packaging is validated offline separately; historical
source/binary hashes are not silently relabelled as the new build.
