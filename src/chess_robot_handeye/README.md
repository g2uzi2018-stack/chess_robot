# Stopped-pose hand-eye calibration

ROS 2 Humble / C++17 / OpenCV 4.5.4. The node owns configuration, sequencing,
stopped-pose acquisition, OpenCV ChArUco/PnP/Park hand-eye solving and held-out
consistency validation. Existing arm and vision nodes retain their responsibilities.
It does not open USB, call the controller SDK, enable hardware, initialize/restore
references, clear errors, control the gripper, or publish production TF.

## Configuration and modes

Copy `config/handeye.example.yaml`. The template deliberately has UNKNOWN board
parameters, no trajectory, disabled execution and `mode: validate`. Its model,
Home and TCP describe the Nova5/AG95 installation, not universal robot defaults.
The manufacturer's pattern or detection must confirm the full marker IDs
and placement, square/marker dimensions, dictionary and legacy layout. Empty
`marker_ids` means ascending IDs from zero, which also needs confirmation.

All length fields are metres, joint/RPY fields radians, and pose quaternions XYZW.
Pose IDs must be unique simple filename-safe strings; role is `train` or `validation`.
`position_m`/`quaternion_xyzw` describe FINAL ACTUAL MODEL TCP in `base_frame`.
They are not board locations or flange coordinates.

* `validate`: configuration parsing/report only; does not use arm/vision interfaces
  or send motion commands, and does not initialize the model. The ordinary rclcpp
  Node parameter/rosout interfaces still exist. Unknown board
  geometry is explicitly reported. Exit 2 means no accepted calibration.
* `plan`: requires a nonempty finite pose list. Sends only `plan_only=true` goals
  to the already running arm. No images are sampled and no solution is accepted.
  Each goal is planned from the current measured pose: this is NOT certification
  of the sequence of transfers or camera visibility. No automatic motion occurs.
* `execute`: requires all three explicit attestations `execution.enabled`,
  `execution.poses_confirmed`, `board.confirmed`, enough train/validation poses,
  valid board parameters and live arm/vision streams. This mode sends real goals
  when `fixture=false`. Keep it disabled until separately authorized and checked.
  Before EVERY execute goal, a fresh exactly matched image/CameraInfo/FrameInfo
  triplet must pass optical-frame, stream health, BGR dimensions/data and rectified
  intrinsics checks. Missing/unhealthy/mismatched vision fails with zero goals at
  the first point. This does not require board detection at the initial pose and
  is not proof of visibility from the target pose. Plan mode skips this preflight.

The current arm's downward-axis constraint is about 0.15 rad (8.6 degrees).
Large tilts can therefore be refused even when a TCP translation passed old Home
tests. This package does not relax that constraint or declare an arbitrary pose
safe. Model/physical collision differences and complete scene validation remain
existing project limitations. A list of poses is NOT a physical reachability claim.

The AGX OpenCV 4.5.4 board implementation uses the legacy ChArUco layout. This
package explicitly rejects `legacy_pattern: false`; CMake also rejects OpenCV
4.6 or newer rather than silently adopting a different layout. It does not ignore
the even-row pattern incompatibility. A different physical pattern needs an
explicit compatible implementation/library and new validation.

## Existing interfaces and exact-target conversion

The arm's PREGRASP operation adds base +Z `approach_clearance_m`. Immediately
before every goal, the node reads the existing arm's parameter service and checks
the expected clearance, flange-to-TCP transform and URDF content. It sends target
Z = configured actual TCP Z minus that verified clearance. It NEVER changes arm
parameters. If this transformed input violates the arm's admission workspace, the
goal is rejected and the run stops: no clearance adjustment or motion bypass.

Movement requires Action SUCCEEDED, `COMPLETED=1`, `executed=true`, valid/fresh
measured final TCP and configured position/orientation tolerance. PLAN_READY=0
and command ACK cannot qualify as arrival. Fresh state must be idle mode 5,
not busy, referenced, matching the configured real/fixture mode. Joint names,
positions, measured velocities and position differences must support stationary
motion. Continuous dwell is established only with advancing original timestamps,
bounded gaps and fresh state/joints. Duplicate/backward feedback does not extend
the dwell. `hardware_certified` is not asserted or required by this orchestrator;
the existing arm's execution gates remain authoritative.

Each point then captures the FIRST eligible post-dwell frame with exact shared
image/CameraInfo/FrameInfo stamp and optical frame, stream_ok, valid BGR8 dimensions,
matching rectified K/P and zero distortion. It uses the closest actual joint/state
snapshot within `pair_tolerance_s`, and requires stationary feedback to advance
through the image time. FK uses the existing twist Model's **flange(q)**, not the
uncalibrated grasp TCP. There is no claim of calibrated exposure/receipt delay or
motion-time synchronization. This procedure is for stopped captures only.

Every error ends the finite run. There is no automatic retry, recovery, resume,
Home return or gripper movement. On an owned pending goal, timeout, abort or
SIGINT/SIGTERM requests cancellation through the same Action. Cancel response
alone is NOT physical stop: the report separately records observed idle/stationary
dwell. An unresolved late goal response is reported as unknown, not safe/stopped.
An operator must resolve an unconfirmed stop through existing arm procedures.

Stop confirmation consumes advancing original JointState samples paired to fresh
ArmState, checks reported velocities AND position differences over receipt time,
and requires a continuous bounded-gap dwell. Callback-detected movement, bad
feedback, duplicate/backward stamps or gaps reset that window; buffered samples
predating the anomaly cannot rebuild it. `stop_proof.csv` records paired stamps,
positions/velocities, anomaly generations, differences and rejection reasons.
Failure to establish the window leaves `physical_stop_confirmed=false` with an
explicit `stop_confirmation_failure`, independently of the cancel response.

## Launch and artifacts

Build without scanning ignored Python environments under `runs/`:

```bash
# Run from the root of your workspace. Existing dependencies must be built/sourced.
source /opt/ros/humble/setup.bash
source install/setup.bash
source install_vision/setup.bash
colcon --log-base log_handeye build --base-paths src/chess_robot_handeye \
  --build-base build_handeye --install-base install_handeye \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install_handeye/setup.bash
ros2 launch chess_robot_handeye handeye.launch.py config_file:=/absolute/reviewed.yaml
```

The launch starts ONLY the orchestrator, never arm/camera/preview services. Choose
the SAME ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY as the existing streams. Verify them
instead of assuming the preview service is running. GUI is not required.

`output_directory` must resolve to a NEW directory with an existing parent; any
existing directory is refused. Relative output and URDF paths resolve against the
configuration YAML directory, independent of the working directory. `model.urdf`
also accepts `package://package_name/resource`; source the package's install first.
No shell/environment/tilde expansion is performed. Copy the template to a writable
scene directory before use. Each point has PNG, readable sample YAML, original
CameraInfo/FrameInfo/ArmState/JointState serialized CDR messages and stationary
proof CSV. Root artifacts contain original `config.input.yaml`, resolved-path
`config.yaml`, copied URDF, source fingerprint, OpenCV
version, per-point results and failure reasons. CDR can be deserialized using the
same ROS message definitions and `rclpy.serialization.deserialize_message`.

Abort through `ros2 service call /chess_robot_handeye/abort std_srvs/srv/Trigger '{}'`
or SIGINT. Wait for the session result; do not infer physical stop from process exit.

`session.yaml` stores `flange_T_camera`, mapping camera points into flange:

```
p_flange = flange_T_camera * p_camera
base_T_board = base_T_flange * flange_T_camera * camera_T_board
```

All training captures are collected before solving. Nonparallel rotation axes,
finite rigid transforms, PnP quality and minimum train/validation counts are checked.
The validation subset never participates in solving. Translation/rotation maxima
compare reconstructed fixed-board poses against a training-only reference.
`accepted=true` means this consistency gate passed; `physical_accuracy_verified`
remains false. A separate physical point/measurement test is still required before
using the result for grasping. No result is automatically written to URDF or TF.

## Tests (synthetic only)

```bash
colcon --log-base log_handeye test --base-paths src/chess_robot_handeye \
  --build-base build_handeye --install-base install_handeye \
  --return-code-on-test-failure
colcon test-result --test-result-base build_handeye --verbose
python3 src/chess_robot_handeye/test/integration.py \
  --template src/chess_robot_handeye/config/handeye.example.yaml \
  --fixture build_handeye/chess_robot_handeye/handeye_fixture \
  --node install_handeye/chess_robot_handeye/lib/chess_robot_handeye/handeye_node \
  --output-parent build_handeye/manual_test_runs --case success
```

All 14 integration cases are registered with CTest and run by `colcon test` above;
the Python command is an optional single-case reproduction. Each run creates a
unique child directory when using `--output-parent`. Integration case options include `plan`, `reject`, `no_motion`, `stale`,
`no_vision`, `camera_unhealthy`, `mismatch`, `cancel`,
`cancel_moving_zero_velocity`, `cancel_duplicate_stamp`, `cancel_backward_stamp`,
`cancel_invalid_feedback`, `cancel_unpaired_feedback`. The harness generates artificial ChArUco
images and model joint feedback, uses isolated `/simulation/` endpoints in ROS
domain 119, and never contacts a robot controller. Run cases sequentially. The
fixture board/poses are synthetic test data, NOT accepted physical configuration.
The fixtures validate orchestration and failure behavior, not real path safety.
The fixture atomically replaces configuration/stats files and publishes an explicit
ready marker after initialization; the harness waits on that contract instead of
reading a partially truncated YAML file or sleeping a guessed startup interval.
It also verifies clean fixture teardown before reporting PASS. Plan's fixture
intentionally publishes no visual messages to prove the visual gate is execute-only.

## Known-board marker refinement (static recognition)

The product detector retains rejected candidates from normal `detectMarkers`.
Only when a native decoded marker belongs to the configured board does it call
OpenCV's standard `refineDetectedMarkers` before ChArUco interpolation, using the
original rectified image, the same board, K and zero D. Detector defaults
(including adaptive threshold constant 7), image size, board geometry/ID mapping,
minimum corners, PnP reprojection and downstream hand-eye/held-out gates are not
changed. No-marker and no-configured-board-anchor inputs fail explicitly.

The noninstalled `handeye_detector_probe` test target calls the actual product
`detectBoard` on saved PNG/capture metadata; it does not initialize ROS or access
USB/robot streams. `test/refine_regression.py` exercises the real saved frame,
an occluded copy, a wrong board, a capacity-valid wrong dictionary, a blank image
and an unrelated marker ID. Example, after an isolated build:

```bash
python3 test/refine_regression.py --probe /absolute/build/handeye_detector_probe \
  --raw /absolute/color.png --capture /absolute/capture.json \
  --output /absolute/NEW_static_recognition_run
```

For the saved 640x480 D405 frame tested on 2026-10-29, the verified plate has
12x9 **squares**, square side 0.015 m, marker outer black side 0.01125 m,
DICT_5X5_100 and legacy layout. These are the static fixture's explicit values,
not replacement defaults for other installations. The C++ product returned 66
corners, RMS 0.241467 px; the occluded frame returned 53, RMS 0.266140 px.
These numbers establish static recognition usability only, not hand-eye,
robot/point positioning or industrial accuracy. Physical deployment remains
subject to the existing configuration and acceptance gates.

## Reuse and field evidence

Read [PORTING.md](PORTING.md) before moving to another scene, robot or camera.
The installed core is available as `chess_robot_handeye::handeye_core`; the node
still uses this repository's action/state/frame contracts. The supported baseline
is ROS 2 Humble, Linux and OpenCV 4.5.x (tested 4.5.4), eye-in-hand, six bounded
revolute joints. It is not an independently certified industrial calibration product.

The [2026-10-30 field report](../../docs/validation/handeye-20261030.md) records
48 captures, held-out consistency and two qualitative physical corner checks.
Those checks do not establish submillimetre physical accuracy or a verified TCP.
