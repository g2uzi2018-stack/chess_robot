# Whole-arm geometric twist definition

Status: implemented in C++ package `chess_robot_twist`; AGX native build and 20
GTest cases passed. This is a rotation-policy model, not real cable mechanics.

## Geometry

FK constructs a centerline from base origin, through joint origins, to the
configured grasp TCP. Coincident consecutive origins collapse to the last
outgoing material frame (Nova5 Link1/Link2 is an important case). Each remaining
span has unit tangent t and a material director d perpendicular to t, attached
to its link. Local director choice is fixed at taught Home, not reselected on
each update. A nonzero flange-to-TCP span preserves distal axial rotation.

Minimal-rotation parallel transport P carries d_i between successive tangents.
The local principal phase is:

alpha_i = atan2(t_(i+1) dot ((P_i d_i) cross d_(i+1)),
                (P_i d_i) dot d_(i+1)).

Parallel tangents use identity transport. Antiparallel tangents have no unique
minimal-rotation axis and are rejected. Zero/nonfinite spans, nonperpendicular
directors and changing topology are rejected. Overall end tangents may oppose
each other; transport proceeds locally along the whole centerline.

This uses the adapted-frame/parallel-transport construction discussed in
[Discrete Elastic Rods](https://www.cs.columbia.edu/cg/rods/index.html), without
simulating stiffness, forces or dynamics.

## Continuous metrics

Principal local phases are unwrapped across a trusted sequence of feedback or
small planned steps. beta_i is continuous local phase minus Home phase.

- Signed net twist: Phi = sum(beta_i), radians.
- Total twist: B = sum(abs(beta_i)), radians. Opposite internal twists do not cancel.
- Geometric energy: E = sum(beta_i^2 / dual_length_i), rad^2/m, where dual length
  is the mean of the adjacent span lengths.

Hard net/total limits are checked independently before any soft objective.
Default policy values are 2*pi for both; these are configurable control choices,
not measured cable capability. Joint hardware/model limits remain separate.
FK geometry itself permits mathematical full-turn counterexamples; a planner
must enforce `withinJointBounds` before admitting executable samples.

No endpoint quaternion can reconstruct lost turns. `Tracker` starts invalid,
requires an explicit near-Home initialization, and becomes invalid on ambiguous
joint/phase jumps. Its owner must also enforce fresh stationary initialization
and invalidate on telemetry gaps. Restart/reconnect/Home arrival never clears
history implicitly. Planning copies the tracker and checks every subdivided
sample, leaving the live feedback tracker unchanged.

The implementation checks finite step-size bounds; it does not claim a formal
continuous-time proof between samples. Sampling, execution tracking margins and
physical stop behavior still require system-level validation.

## Independent checks and native regression

On the actual Nova5 URDF and taught Home:

| Mathematical test | Net | Total |
| --- | --- | --- |
| Base +360 degrees | 360 degrees | 360 degrees |
| Tip +360 degrees | 360 degrees | 360 degrees |
| Base +360, tip -360 | approximately 0 | 720 degrees |
| Joint2-only 0.2 rad change | approximately -0.00000136 degrees | approximately 0.00000136 degrees |

These full-turn tests may exceed physical joint limits and were never executed
on hardware. Generic constructed Bishop-frame bending contributes zero twist.

Other tests cover reversal/unwinding, copy isolation, valid endpoints with an
invalid intermediate state, phase-boundary noise, explicit initialization,
discontinuity invalidation, malformed geometry, transform validation and position
bounds. Evidence: `runs/20261024_arm_root_checks/rod_probe*.json` and
`runs/20261024_arm_integration/twist_{build_final,test}.txt`.
