# Nova5 read-only validation tools

New C++ code; vendor examples are never launched. No enable, motion, safety/configuration write, reset, pause, continue or stop command is implemented. This package is not a motion driver. An operator must independently verify model, firmware, protocol and units before selecting a feedback profile.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select chess_robot_description chess_robot_validation --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

## Evidence and offline checks

Run the registered offline suite after building:

```bash
colcon test --base-paths src --packages-select chess_robot_validation --event-handlers console_direct+
colcon test-result --test-result-base build/chess_robot_validation --verbose
```

This runs the command allowlist/reply parser, feedback frame/sequence fixtures and loopback transport
fault tests. The transport tests cover invalid IPv4/port/timeouts, fragmented
data, incomplete EOF, silent peers, slow trickle deadlines, terminated replies,
oversized replies and the exact 16384-byte reply boundary. They use only loopback
and never contact a controller. Each test has a 20-second runner timeout.
Restrict discovery to `src` so ignored environments under `runs` are not scanned
as candidate ROS/Python packages.

The nonblocking transport checks socket setup errors and uses monotonic deadlines.
`exact` has one budget for the complete read. `query` has one 3000 ms budget for
sending and receiving together, plus a 500 ms maximum wait for each reply byte;
incoming fragments do not extend the total budget. A semicolon completes a reply,
with at most 16384 bytes including that terminator. A complete raw reply is not
proof of command success or a confirmed controller identity/protocol.

Use a new ignored `runs/<run-id>/` directory for each session. Never overwrite prior captures.

```bash
ros2 run chess_robot_validation nova5_readonly --self-test
ros2 run chess_robot_validation nova5_feedback --self-test
ros2 run chess_robot_validation nova5_model_check src/chess_robot_description/urdf/nova5.urdf runs/<run-id>/model
ros2 run chess_robot_validation nova5_readonly --probe <confirmed-controller-ip>
```

`--probe` opens and closes TCP connections to 29999/30003/30004, sends no application bytes and records wall and monotonic elapsed time. It is not proof of robot identity or protocol support. Query mode permits only four exact read commands and profile-specific FK/IK calculations. All other strings are refused before a socket is opened.

```bash
ros2 run chess_robot_validation nova5_readonly --query <ip> common 'RobotMode()'
ros2 run chess_robot_validation nova5_readonly --query <ip> common 'GetAngle()'
```

V3 uses `PositiveSolution(...,user,tool)` and `InverseSolution(...,user,tool)`; V4 uses `PositiveKin` and `InverseKin` with named arguments. V3 near-reference syntax is intentionally not guessed. Unknown profile-specific commands and firmware queries are not sent.

`common` permits only `RobotMode()`, `GetAngle()`, `GetPose()` and `GetErrorID()`;
it makes no firmware-version assertion. Unknown profiles are refused before
opening a socket. Query replies must contain a signed integer controller error,
balanced payload braces and a matching command echo. Raw replies are printed
before parsing. Exit 0 means a well-formed reply with controller error 0; exit 3
means a well-formed controller rejection; exit 2 means invalid input, transport
failure or malformed/mismatched reply. This does not imply motion completion.

If a reply stops before its semicolon, `QueryFailure` retains the received bytes.
The CLI prints the byte count and up to 256 bytes in hex. This exposes plain-text
controller notices such as `Control Mode Is Not Tcp` instead of hiding them
behind a generic read timeout.

## Record and inspect real feedback without ROS publication

```bash
ros2 run chess_robot_validation nova5_feedback_capture <ip> 10 runs/<new-run-id>/actual
```

This C++ tool reads only port 30004 and sends no application bytes. It reuses the
documented common 1440-byte subset from the project's vendor `RealTimeData`
reference: length, sentinel, raw controller timestamp, mode, speed scaling,
actual joint positions/velocities and actual TCP pose. It checks framing, finite
fields and increasing timestamps; a full frame must arrive within 500 ms.
It does not decode firmware-dependent tail fields or identify firmware from a
matching frame. Joint/TCP unit labels follow the vendor schema and should be
cross-checked against the dashboard queries and actual configuration.

The new output prefix creates `.bin`, `.csv` and `.summary.txt` exclusively;
existing files are never truncated. All bytes received, including partial or
invalid frames, go to the binary file before validation. Only validated complete
frames enter CSV. The summary records failure and partial byte counts when
capture fails. Reception median/P95/max intervals describe the host receive
path, not a guaranteed controller sample period or a no-loss assertion.
Capture duration accepts a finite number in (0,120] seconds; completing the last
frame can extend it by up to 500 ms, excluding connection setup. Captures fail on
stream errors, invalid frames, changed/backward timestamps or a 32 MiB byte
budget. Successfully captured data is evidence, not overall hardware acceptance.

Capture-only sequence handling preserves byte-identical duplicate frames in the
raw file, counts them in the summary and omits them from CSV/interval statistics.
A repeated timestamp with changed bytes or a backwards timestamp still fails.
More than 500 ms without a new unique frame fails, including a stream containing
only duplicates. This exception was added after observing a fully identical
repeated frame on hardware. The existing ROS publisher retains its strict
non-increasing-timestamp stop policy.

`nova5_feedback_capture --inspect-raw file.bin` performs the same frame/sequence
checks offline and reports unique and identical duplicate frame counts. It
rejects incomplete trailing frames and empty captures; no network is opened.

## Actual feedback, only after protocol and units are confirmed

```bash
export ROS_DOMAIN_ID=62 ROS_LOCALHOST_ONLY=1
ros2 run chess_robot_validation nova5_feedback <ip> v3 10 runs/<run-id>/actual --confirmed-profile-and-degrees
```

Reads 30004 only, publishes `/nova5/real_joint_states` with joint1..joint6, degrees converted to radians, no effort field. The CSV preserves reception UTC/monotonic ns and controller timestamp **raw**; clock semantics must be confirmed for firmware. Raw 1440-byte packets retain all unparsed fields. Length, sentinel, finite values and monotonically increasing controller timestamps are checked. Stream framing error or >500 ms read delay stops publication. No reconnect or stale republishing. Raw files preserve bad frames for diagnosis. V3/V4 common offsets are the only decoded subset; selecting a profile alone does not validate compatibility.

Run robot_state_publisher with the actual URDF and `joint_states:=/nova5/real_joint_states`; use `config/nova5.rviz`, fixed frame `base_link`. Base and flange/TCP equivalence with the controller remains unverified; `Link6` must not be labelled a validated TCP.

## Explicit synthetic TF check

Start robot_state_publisher with `joint_states:=/validation/synthetic_joint_states`, the URDF as robot_description, `ROS_DOMAIN_ID=62 ROS_LOCALHOST_ONLY=1`, then run `nova5_tf_check runs/<run-id>/model_fk.csv`. This compares robot_state_publisher's TF to offline Eigen FK at 13 synthetic poses. It never publishes to the real feedback topic. This is model plumbing only, not hardware acceptance or collision clearance.

## Preregistered thresholds

- KDL versus direct Eigen composition from the same URDF: <=1e-10 m position norm and <=1e-10 rotation-matrix Frobenius norm. Numerical implementation check, not a vendor accuracy specification.
- Synthetic JointState to TF versus FK: <=1e-8 m and <=1e-8 rad, TF reception age <=0.5 s, exactly 13 poses. Numerical/transport check only.
- Candidate poses: scaled Jacobian min singular value >=0.05, condition <=80; translational rows divided by 0.85 m characteristic length. Engineering screening only; no self/environment collision check and no hardware suitability assertion. Zero is a separate diagnostic, not an acceptance pose.
- Hardware kinematics: report raw residuals and propose 0.5 mm/0.1 deg for model/controller comparison only, pending user acceptance and frame verification; do not turn this proposal into a pass criterion.
- Motion, tracking, speed, acceleration and stop response: no threshold adopted without firmware-specific limits, sampling characterization and operator-approved motion envelope.

No physical velocity limits are installed: the original URDF zero velocity/effort fields remain unchanged. The negative allowlist tests establish that this client blocks all actuation, not that the robot controller enforces speed limits. No claim of physical limit or absolute TCP accuracy validation is made.

## RViz-only scope (user confirmed robot disconnected)

`nova5_rviz_demo <urdf> <output.csv>` publishes only `/validation/synthetic_joint_states` and `/validation/synthetic_flange_path`. No network sockets or controller API are used. It plays a 20 s smooth, closed joint curve with 5 s holds before/after at nominal 50 Hz and logs generated joint positions/analytic velocities, KDL flange pose and scaled Jacobian conditioning. It checks model position bounds and the offline Jacobian criteria only. Physical velocity/acceleration limits, dynamics, collisions and execution accuracy are not validated. The cyan RViz path is the model's Link6 origin, not a measured TCP or a MovL command.
