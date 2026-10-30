# Porting stopped-pose hand-eye calibration

This module is reusable within a ROS 2 system that implements the contracts below.
It is currently an eye-in-hand implementation, not a robot-independent plug-in.
The supported/tested baseline is Linux, ROS 2 Humble, C++17, OpenCV 4.5.x
(tested 4.5.4). OpenCV >=4.6 is rejected at configure time because board layout
compatibility has not been implemented and validated. Eye-to-hand, continuous
motion acquisition and hardware exposure synchronization are not supported.

## What changes between installations

| Change | Work required |
|---|---|
| Same robot/camera, new fixed board location | New scene config and checked poses; keep the board fixed throughout all compared runs; revalidate visibility, reachability and collision clearance. |
| New board | Confirm square count (not inner-corner count), metres, marker outer black side, dictionary, IDs/placement and legacy layout. |
| New camera | Supply the rectified image/metadata contract below, valid intrinsics at the selected resolution, and a new optical frame; rebuild calibration evidence. |
| New mounting or tool/TCP | Update actual model/tool geometry and controller parameters, then recalibrate/revalidate. Previously saved transforms and poses are not transferable. |
| New robot | Implement/adapt the arm contracts and planner semantics below; supply its URDF, Home, frames, limits and measured TCP. Current model requires six bounded revolute joints. |
| Different ROS/OpenCV/platform | Treat as a code port; build and run all offline tests, then perform new controlled physical acceptance. |

Changing the fixed board position does not mathematically change a rigid camera
mount transform, but invalidates mixed-session fixed-board consistency checks.
Changing resolution/intrinsics, robot model or camera mounting requires new
evidence; do not simply copy the old result into production.

## Boundaries and units

`handeye_core` provides config parsing, rigid transforms, ChArUco/PnP detection,
Park solving and held-out validation. It opens no robot/camera connection.
It links OpenCV, yaml-cpp and ament_index_cpp (package resource lookup).
The installed CMake target is `chess_robot_handeye::handeye_core`; see
`test/consumer` for an external installed-header/link smoke test.

The ROS node sequences existing interfaces. It never enables a robot, clears a
fault, restores a winding reference, starts camera hardware or publishes production
TF. Recovery remains with the existing arm node/operator workflow.

Lengths/translations are metres; joints, RPY and angular thresholds are radians;
quaternions are XYZW. `parent_T_child` maps child coordinates into parent:

```
p_flange = flange_T_camera * p_camera
base_T_board = base_T_flange * flange_T_camera * camera_T_board
```

Pose positions/orientations are final model TCP poses in the configured base frame.
URDF joint names are read from the model, rather than assumed to be `joint1` etc.
The model's TCP must agree with the arm node; an approximate model TCP is not a
measured physical finger midpoint.

## Arm adapter contract

Read `chess_robot_interfaces/action/MoveArm.action`, `msg/ArmState.msg` and the
checks in `src/node.cpp` when implementing an adapter. Topic remapping alone does
not change message semantics.

- Action goals use PREGRASP, low-speed profile, `plan_only` and fixture status.
  PREGRASP adds base +Z clearance: this node subtracts the verified clearance
  from the requested target Z. An adapter must implement exactly this behavior,
  or the node's conversion must be explicitly redesigned and tested.
- Parameter service exposes `approach_clearance_m`, `tcp.translation_m`,
  `tcp.rpy_rad`, `urdf_file`. Clearance/TCP and URDF bytes must match the config.
- Arrival requires action SUCCEEDED, COMPLETED, executed=true and fresh measured
  final pose within tolerance. Planning/ACK success is not arrival.
- ArmState and JointState must represent measured feedback, with advancing
  original timestamps, consistent joint names, finite positions/velocities,
  valid reference, idle mode 5 and not busy. The full configured stationary dwell
  must pass both reported velocity and position-change checks.
- Cancellation must have real semantics; client cancellation acknowledgement
  alone is not proof of physical stop. Preserve the controller's existing limits,
  winding history, collision and fault admission rules.

Other robot controllers are not covered by Nova5 acceptance evidence. A robot
adapter must be tested independently before authorizing physical execution.

## Camera adapter contract

Inputs are sensor_msgs Image/CameraInfo and this repository's
`chess_robot_vision/msg/FrameInfo`. They must have identical stamp and matching
configured optical frame, dimensions and stream-health metadata. Use BGR8
**rectified** images with matching rectified K/P and zero distortion coefficients.
Do not zero raw-camera distortion fields without actually rectifying the image.
The node uses stopped-pose host reception timing; hardware exposure alignment
is not certified. A non-D405 camera needs an adapter producing this metadata.

The current `chess_robot_vision` package contains both FrameInfo and the D405
driver, so a source build currently brings librealsense dependencies even if a
different camera adapter is used. Splitting message definitions from that driver
is a future packaging improvement, not a capability of this release.

## Scene workflow

1. Build/supply `chess_robot_interfaces`, `chess_robot_vision`,
   `chess_robot_twist` and normal ROS/OpenCV/yaml dependencies. Source those
   installations, then follow the handeye README build commands. For the Nova5
   example also build/source `chess_robot_description` so its package URI resolves.
2. Copy `config/handeye.example.yaml` into a writable scene directory. Replace
   the Nova5-specific model/Home/TCP, topics and frames as appropriate. Relative
   filesystem paths use this YAML's parent directory; URDF accepts `package://`.
   Output must be new and its parent must exist. No user/home path is required.
3. Confirm board dimensions/pattern from measurement or manufacturer data and
   observed IDs. Keep `execution.enabled=false`, `poses_confirmed=false` until
   that scene's path and geometry have been checked. Populate new reachable
   training and held-out poses; vary at least two nonparallel rotation axes while
   respecting robot limits and visibility. Repeating nearly identical views
   does not replace diverse geometry.
4. Run `validate`, inspect its report (exit 2 means no accepted calibration).
   It performs no arm/vision IO, but initializes an ordinary ROS node. It does not
   prove URDF readability, robot reachability or board visibility.
5. Run `plan` against the intended arm adapter. It sends plan-only goals from
   the current measured state; this does not prove all sequential transfers safe.
6. After scene clearance and authorization, run `execute`. Retain raw artifacts,
   config snapshots, model and source fingerprint. Keep the board and mount rigid.
   Inspect failures instead of weakening a freshness or stationary threshold.
7. Compare independent runs, inspect held-out residuals and validate multiple
   physical targets with a measured TCP before deploying. `accepted=true` means
   internal consistency; it is not a physical accuracy certificate. No generic
   constant offset can be inferred from a scalar residual.

`runs/` holds local experiment processes, poses and recovery history. They are
ignored and not a portable operator API. Do not copy process IDs, reference files
or old motion points to a different scene. Historical `docs/validation` records
evidence, not defaults. The dated candidate transform is not auto-loaded.
The continuous board/target overlay used during physical inspection currently
lives in local diagnostic scripts. The installed handeye node records captures
and results; it does not publish a continuous preview overlay. Porting that live
inspection UI requires packaging a separate detector publisher for the preview's
configured image topic.

## Verification before reuse

Run `colcon test` and inspect `colcon test-result`; the 14 synthetic integration
cases use loopback ROS domain 119 and `/simulation/` endpoints without hardware.
Avoid concurrent unrelated nodes in that test domain. The installed consumer
smoke test can be built with `cmake -S src/chess_robot_handeye/test/consumer -B
build_consumer` after sourcing the new install, then `cmake --build build_consumer`
and `./build_consumer/consumer`. Successful offline tests do not replace the new
installation's physical acceptance.
