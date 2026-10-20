# Nova5 read-only validation tools

New C++ code; vendor examples are never launched. No enable, motion, safety/configuration write, reset, pause, continue or stop command is implemented. This package is not a motion driver. An operator must independently verify model, firmware, protocol and units before selecting a feedback profile.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select chess_robot_description chess_robot_validation
source install/setup.bash
```

## Evidence and offline checks

Use a new ignored `runs/<run-id>/` directory for each session. Never overwrite prior captures.

```bash
ros2 run chess_robot_validation nova5_readonly --self-test
ros2 run chess_robot_validation nova5_feedback --self-test
ros2 run chess_robot_validation nova5_model_check src/chess_robot_description/urdf/nova5.urdf runs/<run-id>/model
ros2 run chess_robot_validation nova5_readonly --probe <confirmed-controller-ip>
```

`--probe` opens and closes TCP connections to 29999/30003/30004, sends no application bytes and records wall and monotonic elapsed time. It is not proof of robot identity or protocol support. Query mode permits only four exact read commands and profile-specific FK/IK calculations. All other strings are refused before a socket is opened.

```bash
ros2 run chess_robot_validation nova5_readonly --query <ip> v3 'RobotMode()'
ros2 run chess_robot_validation nova5_readonly --query <ip> v3 'GetAngle()'
```

V3 uses `PositiveSolution(...,user,tool)` and `InverseSolution(...,user,tool)`; V4 uses `PositiveKin` and `InverseKin` with named arguments. V3 near-reference syntax is intentionally not guessed. Unknown profile-specific commands and firmware queries are not sent.

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
