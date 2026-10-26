# Arm controller (ROS 2 Humble, C++17)

This package delivers software for a six-axis Nova5 arm. It defaults to passive
`read_only` and loopback IPv4; in this default mode it sends no enable/reset/configuration commands and
never initializes the cable reference automatically. The integrated source has
been built and tested on AGX / ROS 2 Humble. See the delivered acceptance report
for exact evidence. Hardware ServoJ/Stop certification remains outstanding.
The launch now loads a sampled geometry/Jacobian guard for the supplied Nova5,
camera and AG95 model. It includes the user-specified table plane and documented
adjacency exclusions. It is not continuous collision checking or a complete
scene model. See `docs/validation/business-node-2026-10-25.md` for real tests.

## Software repair baseline, 2026-10-25

Firmware reported by the operator is `4.6.2.4-stable-d9efa2ce0-202509261512`.
The pinned official C++ V4 API is now used for FK/IK/ServoJ/Stop command encoding
and `CFeedbackData` decoding. See `third_party/dobot_v4/UPSTREAM.json`,
`LOCAL.patch`, and `README.md` for provenance and minimal deviations.
Full evidence: `docs/validation/software-repair-20261025.md` in the workspace.

Motion stays paused with `motion.paused=true` and `hardware.enable=false`.
There are two configurable modes: low = 0.708 rad/s, 1.416 rad/s²;
high = 2.36 rad/s, 4.72 rad/s². Default is `profiles.default=low`.
These are the pinned official Nova5 ROS2 planning limits and its 0.3 scaling,
not a measurement of this controller's reproduction/reduced limits. Confirm those
limits before any motion. 3 rad/s exceeds this official planning configuration.
No SpeedFactor/VelJ/AccJ command is injected into ServoJ execution: streaming
velocity comes from the checked trajectory timing; SDK percentage is not rad/s.
Legacy profile names remain accepted by their matching overlays.

`joint_states`, `state.header`, `state.tcp` and measured action final TCP use
the ROS time sampled immediately after a complete feedback frame is received.
The same snapshot supplies joints and FK; controller time is used only for
ordering/duplicates. Publishing cannot freshen a receipt timestamp.

## Finite Home -> exact TCP -> Home acceptance

Use `tools/testing/arm_roundtrip.py` and its `arm_roundtrip_README.md` from the
workspace root. `testing.home_roundtrip_precheck=true` with zero approach clearance
keeps the existing action schema while checking every outward candidate's predicted
return to formal Home using copied winding history. It never executes that return
implicitly. The tool confirms both separately measured legs and keeps a finite
26-point / at-most-38-target ledger, CSV and 3D status plots.

The tool accepts source-backed controller limits or an explicit human command policy.
The current user policy requests high 1 / low 0.3 rad/s and 1 / 0.3 rad/s² planning
caps; these are not hardware-limit certification or a claim of measured speed.
Fresh actual low/high qualification is required per source/binary/controller epoch.
Existing arrival/stop tolerances and all protections remain active.

Explicit `reference.power_cycle_handoff=true` preserves the complete recorded winding
history across a declared power-only epoch after the original join/bounds/step/twist
checks. Both flags default false. Only when `controller.enable_after_history_restore=true`
is also explicitly set, a successful RestoreReference callback may check official
GetErrorID for empty alarms and call official EnableRobot for mode4; it never calls
ClearError. Enable ACK uses its own default 5 s budget. The client then requires
reference_valid, fresh mode5 and the original stationary dwell before low HOME.
No automatic reference initialization/reset or fault trajectory replay is introduced.

Latest physical continuation is BLOCKED: low Home passed, but new-epoch low qualification
failed on a 146.428485 ms feedback receipt gap; Stop ACK and fresh stationary dwell
were confirmed. No new target roundtrip or boundary refinement passed. The original
14 successful targets remain historical evidence; they do not qualify the new epoch.
See `docs/validation/home-roundtrip-20261026/REPORT.md`; the 10/25 report is historical.

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
  Wire numbers use decimal notation; the real controller rejects exponent
  notation in near-zero Euler angles. Commands share one ordered connection:
  this controller rejects a second command session from the same host. Stop
  uses that same session, with a separate acknowledgement budget. A valid
  controller rejection preserves the session. Ambiguous replies lock further
  queries/motion without reconnecting; the original socket remains for Stop.
  Real ServoJ uses a bounded asynchronous FIFO: six pending writes by default,
  exact echoed commands/increasing queue IDs, 150 ms per acknowledgement.
  A 125 ms reply fits at 30 ms send cadence; a deadline/overflow still fails
  motion immediately. A bounded Stop-only drain may use an additional
  controller.timeout_ms. On broken FIFO, Stop is sent once on the original
  socket and only its exact acknowledgement is accepted within the Stop budget.
  Old complete replies are only audited. No automatic reconnect/replay occurs;
  a physically lost connection cannot guarantee a successful Stop and fails shut.
  Executor still requires fresh mode=5 stationary feedback dwelling to verify it.
  Framing retains coalesced/partial text and caps replies at 16 KiB. Feedback
  decoding copies into the official LP64 structure with compile-time size/offset
  assertions and validates the common 1440-byte header/finite fields.
  Equal controller timestamps are discarded without advancing freshness/history;
  regression invalidates trust. No controller clock synchronization is assumed.
  Exploration defaults ServoJ horizon to the send period (30 ms), independently
  of ACK budget. TCP_NODELAY/per-receive QUICKACK remain transport patches.
* `Model`/`Tracker` from `chess_robot_twist`: authoritative URDF geometry and
  whole-arm geometric rotation policy; the exported target is linked directly. Measured
  feedback alone advances the live tracker under a short mutex. Planning copies
  its full continuous history; invalid geometry/phase/joint jumps fail explicitly.
  No joint angle wrapping or turn guessing occurs. All planner and executor
  samples also call `withinJointBounds`; mathematical model FK is not a joint
  bound policy. Signed/total twist are radians; energy is geometric rad²/m.
  `/arm/restore_reference` explicitly replays original binary feedback audits
  from a known stationary Home, preserving unwrapped local phases. It requires
  idle admission, no latched fault, an explicit no-unobserved-motion attestation,
  complete monotonic frames, valid recorded bounds/twist, and a fresh stationary
  current endpoint within 0.001 rad of the last recorded endpoint with advancing
  controller time. Gaps between stopped sessions are attested, never silently
  reconstructed. The caller must use matching URDF/Home/TCP provenance. It
  cannot overwrite a valid live reference and never initializes at the current
  non-Home pose. A 330° replay test verifies it does not collapse to −30°.
* `Planner`: checked PTP transfers/Home and incremental seeded IK for vertical
  descent/retreat. Current and Home IK seeds are candidates; both SDK FK(IK) and
  URDF FK must agree with the requested TCP. Checked candidates prioritize total
  twist, then absolute net twist, then duration. Values within the 5° neutral
  band are equivalent; leaving the current-seed incumbent requires more than
  2° angular improvement, or (for equivalent twist) more than 1 s time saving.
  These incumbent deadbands suppress switching from numerical noise without
  mutable planning state. Legacy cost weights remain accepted but have no ranking
  effect. Tests cover a slow-profile, FK-verified 330° / -30° equivalent pair,
  noisy near-neutral comparisons, and an unsafe unwind candidate. Hard path limits remain
  mandatory regardless of the preference. There is no hidden lateral or orientation transfer
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
  and fresh stationary idle feedback must verify Stop. Stop can wait behind a
  bounded in-flight query; its total deadline includes command connection/I/O
  and acknowledgement budgets plus the measured dwell. A stop verification failure
  returns STOP_FAILED even when the ROS terminal status is canceled. Passive
  monitoring and plan-only jobs never own motion and never issue Stop.
* `node.cpp`: ROS transport only, one joined task worker and one joined feedback
  worker; the driver owns a joined reply reader for each active stream. There
  are no detached threads. Parameter values are immutable after startup.
  Feedback monitoring remains active while the task worker verifies shutdown stop.

`/arm/move` operations are PREGRASP=0, DESCEND=1, RETREAT=2,
GRASP_SEQUENCE=3, HOME=4. An empty profile selects `profiles.default` (low);
`conservative` remains a compatible low-mode alias.
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
FAULT and refuses goals and reference initialization. Establish real stationary
idle feedback and resolve the failure before restarting with an explicitly
verified Home reference. A fault cannot be cleared by submitting Home.

## Bounded hardware commissioning

`config/commissioning.yaml` is an overlay for explicit local acceptance. Its saved
`hardware.enable` remains false and `motion.paused` defaults true. Backend `commissioning` uses the same production
Planner, V4 driver and Executor, while `hardware_certified` remains false. It
does not set the production verification booleans to true.

Commissioning requires a loaded GeometryGuard, joint displacement from Home at
most 0.12 rad and TCP radius at most 0.04 m (template: 0.1 rad / 0.03 m),
freshness at most 0.1 s and net/total twist at most 20°. The old debug-only speed
ceiling and its derived instantaneous velocity trip are removed. Low/high timing
uses the official limits above; measured tracking, bounds, geometry, twist,
freshness, arrival and Stop checks remain mandatory. Tracking uses the existing
normal 0.15 rad tolerance rather than the old slow-only 0.005 rad setting;
stationary thresholds/dwell are not raised. This is not physical certification.

`backend=exploration` remains an uncertified geometry-bound mode with its existing
near/inner/middle/outer/full envelopes and global twist protections. Low/high
aliases use only the near envelope; choosing a speed does not enlarge geometry.
Legacy stages now use low timing. Servo horizon defaults to the 30 ms cadence,
with aheadtime=100 and gain=200 retained. Saved enable is false and motion paused.
The explorer advances recorded batches only after feedback-confirmed setup
points, follows a fixed coverage checklist, records planning rejects, and aborts
on execution faults. Checked waypoints can change a transfer path; saved failed
trajectories are never replayed. See the operation guide and latest report.

The optional `safety.model_file` loads the geometry exported by
`tools/analysis/workspace_analysis.py --probe-only`: table z=-0.010 m with 20 mm
clearance, moving robot plus the 21-opening gripper envelope, 10 mm nonadjacent
group separation, 5° joint margin, normalized Jacobian sigma_min >=0.05 and
condition <=80. Fixed/tool and directly adjacent groups are excluded. FK, TCP
and joint bounds must match the live node model. Regenerate the file after model
or attachment changes. The geometry file's provenance and exclusions are the
same as `docs/analysis/arm-workspace-2026-10-24.md`.

`audit.directory` optionally records real command/reply latency and raw feedback
for acceptance. It must name a new directory with an existing parent. Raw
feedback records consist of an int64 host UTC nanosecond receipt followed by
the 1440 controller bytes, little endian. It is disabled by default.

## Sampling and remaining acceptance

Defaults are conservative software inputs, not certified operating limits.
Transfer geometry is a straight joint-space path. Cartesian IK uses 2 mm
vertical increments and rejects branch steps over 0.025 rad. C1 Hermite
interpolation and a single quintic scalar time law produce fixed 30 ms samples;
finite-difference velocity and acceleration checks, including zero velocity
before/after each segment, iteratively dilate time until all profile/cap limits
pass. The resulting path is rechecked for Cartesian line/attitude tolerance,
twist and geometry, so smoothing cannot bypass any constraint. Every timed sample
is validated against bounds, workspace and twist. Downward-axis policy applies
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

Production hardware execution requires backend=hardware AND all five booleans under
`hardware` true. All default to false. Setting them is an operator attestation,
not automatic certification: verify TCP calibration, joint/software limits,
workspace, physical cable reference/twist range, SDK/URDF FK equivalence, actual
ServoJ timing/ACK latency, interpolation/tracking, Stop behavior/latency and
command ownership on the target controller before any physical acceptance.
ServoJ support remains subject to the measured local acceptance scope. An
ordered persistent transport avoids per-command connection setup. The bounded
reply FIFO tolerates ACK tail latency independently of the write cadence; missed
write deadlines, acknowledgement limits and feedback faults still stop execution.
No native MovJ/MovL bypass is offered. This package cannot prevent commands from
other controller clients; deployment must enforce exclusive motion ownership.
