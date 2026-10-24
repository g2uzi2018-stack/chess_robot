# Arm controller (ROS 2 Humble, C++17)

This package delivers software for a six-axis Nova5 arm. It defaults to passive
`read_only` and loopback IPv4, sends no enable/reset/configuration commands, and
never initializes the cable reference automatically. The integrated source has
been built and tested on AGX / ROS 2 Humble. See the delivered acceptance report
for exact evidence. Hardware ServoJ/Stop certification remains outstanding.
There is no collision environment or ACM. A geometric plan is not collision
clearance.

## Build and offline validation on AGX

From the workspace root, with ROS 2 Humble and the dependencies in
the package.xml files available:

```bash
bash src/chess_robot_arm/scripts/validate_target.sh
```

The script builds the supplied twist package, interfaces and arm, runs their
selected arm/interface tests, and reports colcon results. It does not launch a
node, contact a controller or send hardware commands. Tests use the source URDF,
actual FK with finite-difference numerical IK, injected transports, and IPv4
localhost fixtures. Test source covers malformed/fragmented replies and retained
partial notices, SDK unit/rotation conversions, duplicate/backward feedback,
reference lifecycle, capture/pose/config validation, task admission/cancellation,
late/duplicate confirmations, rod metrics, sampled path policy, velocity and
acceleration caps, and disabled hardware producing zero actuation calls.

To launch against a **separately authorized read-only** controller or a protocol
fixture (not part of the validation script):

```bash
source install/setup.bash
ros2 launch chess_robot_arm arm.launch.py controller_ip:=127.0.0.1
ros2 topic echo /arm/state
ros2 service call /arm/initialize_reference chess_robot_interfaces/srv/InitializeReference '{}'
ros2 action send_goal /arm/move chess_robot_interfaces/action/MoveArm \
  '{operation: 4, profile: conservative, plan_only: true}' --feedback
```

Reference initialization succeeds only with fresh stationary feedback dwelling
near the operator-taught Home. Without such feedback, the example fails explicitly;
it does not synthesize joint states. Home above is a plan-only operation and
needs no capture pose. Pose operations require a current nonzero ROS capture
stamp, normalized quaternion and `base_link` frame. For an actual perception
message use the action client API to forward its original stamp and pose; do not
replace the capture time with the send time. No arbitrary target coordinates are
provided as purported safe examples.

## Architecture and contracts

* `driver.hpp`, `driver.cpp`, `net.hpp`, `dashboard_reply.hpp`, and
  `feedback_frame.hpp`: typed V4 transport and framing. SDK only uses user=0,
  tool=0 (base and flange); all configured TCP conversions are local. Internal
  lengths/angles are metres/radians, SDK positions are mm/degrees with rotation
  `Rz * Ry * Rx`. Every query has a bounded whole-operation timeout, error and
  matching-echo check. Unterminated controller notice bytes remain diagnostic.
  A new connection per query isolates partial replies and lets Stop use an
  independent connection. Read feedback is 1440-byte common-subset framing on
  30004. Identical duplicate packets never refresh host freshness; altered equal
  or regressing controller timestamps invalidate trust. No controller clock
  synchronization is assumed. Reconnection never restores reference validity.
* `Model`/`Tracker` from `chess_robot_twist`: authoritative URDF geometry and
  whole-arm framed-rod twist; the exported target is linked directly. Measured
  feedback alone advances the live tracker under a short mutex. Planning copies
  its full continuous history; invalid geometry/phase/joint jumps fail explicitly.
  No joint angle wrapping or turn guessing occurs. All planner and executor
  samples also call `withinJointBounds`; mathematical model FK is not a joint
  bound policy. Signed/total twist are radians; energy is geometric rad²/m.
* `Planner`: checked PTP transfers/Home and incremental seeded IK for vertical
  descent/retreat. Current and Home IK seeds are candidates; both SDK FK(IK) and
  URDF FK must agree with the requested TCP. Candidates are selected using configured positive weights for duration and
  endpoint absolute net twist, total twist and geometric energy. Hard path limits
  remain mandatory regardless of these preferences. There is no hidden lateral or orientation transfer
  in DESCEND/RETREAT. They require current XY and attitude already aligned within
  configured tolerances, preserve actual model XY/attitude exactly as the desired
  line, and check every interpolated FK against it. The actual arm remains subject
  to configured FK/servo tolerances, not a mathematical exact-path guarantee.
* `State`, `TaskGate`, `Executor`: explicit idle-only Home reference, atomic task
  admission, exact UUID/phase matching confirmation, cancellation, bounded
  execution and measured dwell completion. Queue ACK is never arrival. Fresh
  feedback must satisfy final joint/TCP error, measured speed, idle mode 5 and
  dwell. The common feedback subset does not validate a firmware-dependent queue
  completion offset: idle mode plus measured arrival is the supported completion
  evidence. An owned ambiguous motion send is stopped on fault/cancel/shutdown,
  and fresh stationary idle feedback must verify Stop. A stop verification failure
  returns STOP_FAILED even when the ROS terminal status is canceled. Passive
  monitoring and plan-only jobs never own motion and never issue Stop.
* `node.cpp`: ROS transport only, one joined task worker and one joined feedback
  worker; no detached threads. Parameter values are immutable after startup.
  Feedback monitoring remains active while the task worker verifies shutdown stop.

`/arm/move` operations are PREGRASP=0, DESCEND=1, RETREAT=2,
GRASP_SEQUENCE=3, HOME=4. Profile names are mandatory (`conservative` by default).
PREGRASP equals requested grasp position plus base +Z clearance with unchanged
orientation. GRASP_SEQUENCE plans all three segments in advance and executes
pregrasp → descent → WAIT_GRIPPER → retreat. `/arm/confirm_grasp` requires the
exact action UUID hex string published in action feedback and `/arm/state`.
Failure/timeout ends the task without retreat. It never sends gripper commands.
Plan-only sequences return the geometric concatenation with no gripper waiting
interval represented in trajectory time; that trajectory is informational and
must not be replayed as a replacement for this task state machine.

Successful plan-only returns PLAN_READY, executed=false, and the sampled joint
trajectory. It does not assert movement completion. Successful motion returns
COMPLETED only after measured arrival. `executed` means motion was attempted,
including ambiguous network outcomes, and is true on later task failure after a
previous segment moved. Final TCP is URDF model FK of real measured joints (not
an independent calibrated Cartesian sensor) and has an explicit validity bit.
Action rejection has standard ROS rejection semantics with a diagnostic log;
typed error codes are returned for accepted jobs. Busy goals are rejected rather
than queued. `/arm/joint_states` publishes only fresh real controller feedback.
For protocol-fixture acceptance only, `test_fixture=true` requires IPv4
127.0.0.1, reports simulated=true, and changes every endpoint to
`/simulation/arm/...`. It cannot publish fixture data on the real arm endpoints.
The fixture is external to this C++ node; there is no synthetic joint generator
in the production driver. Fixture state never reports hardware_certified=true.

If Stop acknowledgement or measured stop verification fails, the node latches
FAULT and refuses goals and reference initialization. On-site recovery followed
by a node restart is required. A fault cannot be cleared by submitting Home.

## Sampling and remaining acceptance

Defaults are conservative software inputs, not certified operating limits.
Transfer paths use quintic joint blend geometric vertices with maximum nominal
0.025 rad spacing; the quintic derivative can increase adjacent spacing, and the
tracker's independent 0.1 rad step gate remains authoritative. Cartesian IK uses
2 mm vertical increments and rejects branch steps over 0.025 rad. Integer time
dilation and subdivision preserve every vertex and fixed 30 ms ServoJ periods.
Finite-difference velocity and acceleration checks include zero velocity at both
segment boundaries. The dilation treats interpolation corner acceleration
conservatively as inverse subdivision count. Samples, including timed inserts,
are validated against bounds, workspace and twist. Downward-axis policy applies
to transfers to pregrasp and all vertical samples; Home PTP may change attitude.
There is no claim of continuous collision checking, continuous extrema proof,
or exact physical path tracking between samples. Tracking deviations and schedule
overruns abort. Total allocation is bounded by a configured sample cap.

`config/home.yaml` preserves **exactly** the six operator-taught radians from
`config/arm/home.yaml`, dated 2026-10-24. `config/arm.yaml` contains nominal URDF
flange→TCP (+0.20 m flange Z, fixed RPY) and +X grasp approach. This is not a
physical calibration. Workspace and caps require operator verification; URDF
velocity/effort zeros are never used as operating limits. The 0.25 m approach
clearance is a configurable task parameter, not a clearance measurement at Home.

Hardware execution requires backend=hardware AND all five booleans under
`hardware` true. All default to false. Setting them is an operator attestation,
not automatic certification: verify TCP calibration, joint/software limits,
workspace, physical cable reference/twist range, SDK/URDF FK equivalence, actual
ServoJ timing/ACK latency, interpolation/tracking, Stop behavior/latency and
command ownership on the target controller before any physical acceptance.
ServoJ support exists in code but remains uncertified. Per-command connection
setup may exceed 30 ms on a controller and will cause schedule-overrun failure;
measure this before enabling, and use a validated persistent transport if needed.
No native MovJ/MovL bypass is offered. This package cannot prevent commands from
other controller clients; deployment must enforce exclusive motion ownership.
