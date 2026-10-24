# Whole-arm geometric twist library

C++17 ROS 2 library; no node, network client or actuation. It models the arm as a
framed geometric centerline from the chosen base through joint origins to TCP.
Coincident origins retain the final outgoing frame. Material directors are fixed
in link-local coordinates at the operator-taught Home.

Minimal-rotation parallel transport measures local twist independently of bending.
Continuous tracking retains full rotations relative to Home. Metrics are signed
net twist, sum of absolute local twist, and sum of squared local twist divided by
dual span length. This is a geometric rotation policy, not cable force simulation.

`Model` performs FK geometry, while joint position bounds are checked explicitly
with `withinJointBounds`. Keeping these separate permits full-turn mathematical
counterexamples; such tests are not executable robot targets. `withinLimits`
enforces the independent configured net and total twist bounds.

`Tracker` starts invalid and requires an explicit near-Home initialization. It
does not infer missing turns, reset on Home arrival or recover after discontinuity.
The owning controller must enforce fresh stationary feedback during initialization
and invalidate on feedback gaps. Planning uses a copy and subdivisions within its
joint/phase step limits. Ambiguous phase jumps, antiparallel tangents and invalid
geometry fail explicitly. A nonzero TCP offset is required to observe distal spin.

ROS target: Humble, Eigen3 and urdf. Public target:
`chess_robot_twist::chess_robot_twist`; header `chess_robot_twist/model.hpp`.

The math tests use a copy of the existing project Nova5 URDF from
`chess_robot_description`; its original vendor provenance is documented in
`reference/dobot/VENDOR_SOURCES.json` in the complete project. Missing mesh files
are not loaded by this FK-only library. No collision-clearance claim is made.

```bash
colcon build --base-paths src --packages-select chess_robot_twist
colcon test --base-paths src --packages-select chess_robot_twist --event-handlers console_direct+
colcon test-result --test-result-base build/chess_robot_twist --verbose
```
