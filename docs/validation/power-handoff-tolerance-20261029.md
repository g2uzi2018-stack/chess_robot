# Bounded final power-cycle endpoint recovery — 2026-10-29

Historical implementation-stage snapshot. Later field/release evidence is indexed
in [the 2026-10-30 report](handeye-20261030.md); pending statuses below describe
this revision at its original freeze time. They are not current recovery permission.

Implementation and offline validation only. No physical commands were issued; no
existing services were started/stopped; the production install was not replaced.
Base: `82bd7965985238f5b991131f723c575ff1f16819`, branch
`feat/ag95-description-20261023`. Pending independent review and deployment.

## Change and scope

The operator declared a power-only restart without unobserved movement. Two
read-only measurements agreed with each other but differed from the original
stopped historical endpoint by at most **0.00269977955507 rad (0.154685974121°)**.
The original fixed final join bound was 0.001 rad, so restoration was refused.

New `reference.power_cycle_endpoint_tolerance_rad` defaults to **0.001 rad**.
It must be finite and strictly positive, and at most
`min(0.01 rad, reference.home_tolerance_rad, reference.max_joint_step_rad)`.
The hard 0.01 rad cap is 0.573° and smaller than the existing default Home and
step envelopes; it is a bounded recovery policy, not an encoder accuracy claim.
For this measured endpoint only, an explicit 0.005 rad (0.286°) setting covers the
observed disagreement while remaining below the cap.

The configured value applies only to the original final live `power_handoff`
branch: explicit `reference.power_cycle_handoff=true`, current mode 4/5, both
historical and live endpoints within the existing configured Home envelope.
The original fresh/stationary/dwell and stopped/fault-history checks still apply.
Ordinary live joins, all historical file joins and the explicitly named old
power epoch join keep the original **0.001 rad** bound. No service schema changes.

Original audits replay from known stationary Home. The original tracker is then
advanced once to current measured joints and checked against the existing twist
hard limits before committing. Home configuration, arrival/tracking/collision
checks and automatic zeroing behavior were not changed. Reaching Home does not
substitute for restoring winding history. Endpoint agreement cannot prove the
operator's no-unobserved-motion attestation; the attestation remains mandatory.

`State::restore` now returns its diagnostic for the existing service `detail`.
This reports the six live-minus-history joint deltas, maximum delta, effective
bound, qualifying power-handoff branch, whether an expanded join actually
committed, and the historical/restored net and total twist. Failure diagnostics
never claim that an expanded recovery committed. Audit file I/O remains outside
the state mutex; the small diagnostic is formatted in memory.

Two equivalent warning cleanups accompany the change: initialize empty
`Plan::selection` explicitly, and represent missing recorded-hold time as zero
(all accepted recorded host timestamps are strictly positive).

## Files

- `src/chess_robot_arm/include/chess_robot_arm/core.hpp`: parameter and internal return value.
- `src/chess_robot_arm/src/core.cpp`: validation, scoped final join, diagnostics and warning cleanups.
- `src/chess_robot_arm/src/node.cpp`: immutable typed ROS parameter declaration.
- `src/chess_robot_arm/config/arm.yaml`: default false handoff / strict 0.001 rad example.
- `src/chess_robot_arm/test/test_arm.cpp`: 13 new focused offline cases.
- `src/chess_robot_arm/test/restore_reference_offline.cpp`: test-only executable; no Driver/socket object; never installed.
- `src/chess_robot_arm/CMakeLists.txt`: test-only executable target.
- `src/chess_robot_arm/README.md`: policy and diagnostics.
- This validation report.

Existing handeye changes, `.gitignore`, old analysis scripts, all original audit
bytes and production binaries remain outside this write scope.

## Reproduction

Run from `/home/kuang/workspace/chess_robot`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon --log-base runs/power_handoff_20261029/log build \
  --packages-select chess_robot_arm \
  --build-base runs/power_handoff_20261029/build \
  --install-base runs/power_handoff_20261029/install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  --event-handlers console_direct+
colcon --log-base runs/power_handoff_20261029/log test \
  --packages-select chess_robot_arm \
  --build-base runs/power_handoff_20261029/build \
  --install-base runs/power_handoff_20261029/install \
  --event-handlers console_direct+
colcon test-result --test-result-base runs/power_handoff_20261029/build/chess_robot_arm --verbose
python3 runs/power_handoff_20261029/node-loopback.py
python3 runs/power_handoff_20261029/real-history-offline.py
```

The localhost harness uses ephemeral localhost ports, `ROS_DOMAIN_ID=78`,
`ROS_LOCALHOST_ONLY=1`, `test_fixture=true`, read-only backend and paused motion.
It creates and terminates only its own two isolated test node processes.
The original production preview/camera services are untouched.

The real-history utility reads the 13 SHA-bound predecessor audits from the
original `physical200_02/results.json`, plus that run's final audit (14 total),
and keeps the original declared `physical_continue01` epoch join. Measured live
joint input comes from the existing `prelower-check.json`. Repeated synthetic
in-memory endpoint snapshots keep only this offline fixture fresh while the
original large audits replay. They are explicitly labelled test snapshots and
are never appended to, or substituted for, original audit records.

## Results

- Release build: success, **zero compiler warnings**.
- Entire affected arm suite: **75 tests, 0 failures/errors/skips**; includes all
  62 existing cases and 13 new recovery cases.
- New tests cover default refusal of 0.003 rad, explicit 0.005 rad recovery and
  exact preserved/advanced nonzero tracker metrics (about 0.01 → 0.013 rad),
  unchanged ordinary/non-Home join bounds, rejected invalid/too-large settings,
  stationary/freshness/dwell/idle requirements, file and recorded epoch mismatch,
  known initial Home, required attestation, and post-advance twist hard limits.
- Localhost ROS service integration: **4 cases passed**, 0 dashboard commands,
  0 physical motion commands; default refusal, explicit recovery and required
  attestation verified through the unmodified RestoreReference service schema.
- All **14 original audit SHA256 values unchanged** before/after offline replay;
  all 13 pre-existing manifest SHA256 values also match.
- Real measured endpoint: default 0.001 rad refuses; explicit 0.005 rad accepts
  with maximum delta **0.00269977955507 rad**.
- Real-history total twist **0.00000578368858689** → **0.000716014413777 rad**;
  net **−0.00000280427711824** → **0.000694917184516 rad**. Original winding
  state was retained and advanced to measured q, never reset.

Evidence under `runs/power_handoff_20261029/`: `final-build.log`,
`all-tests.log`, `test-result.log`, `node-loopback-results.json`,
`real-history-offline-results.json` (14 unchanged audit hashes),
`source-and-binary-sha256.json`, `implementation.patch`, plus reproducible harnesses.

## Binary and source identity

Isolated `install/chess_robot_arm/lib/chess_robot_arm/arm_node` SHA256:
`25b13dccc17a24e08ee920edf0d2b28dcebc75ef0cd38de46e8a3076f84ccaf1`.

Original production `install/chess_robot_arm/lib/chess_robot_arm/arm_node` remains:
`1b5cf1d508bd49afc9b9608c3503940c12da7742a935146945d9d3c1fa848443`.

Source file SHA256:

- `src/chess_robot_arm/CMakeLists.txt`: `b121f6a90b38e0bf9f2c01a9c5e4985eac2a68b625087a1dbc1dacc381b7ef60`
- `src/chess_robot_arm/README.md`: `f5a73accf991e367d67afa0a3470f9f0fa7bc0c121b0c759ddb47adb3fad93a4`
- `src/chess_robot_arm/config/arm.yaml`: `105c6ad8e1d49fdcfa61d20474e53314e5ae93ed65e6f6365f20a179ec34f615`
- `src/chess_robot_arm/include/chess_robot_arm/core.hpp`: `fff0471911fb60e144fbe985015ab77ba66da31b00943eb51d25ecb1062b845b`
- `src/chess_robot_arm/src/core.cpp`: `36fd4af1327a56dbd811f437ef6c8dc9f26c45a7e847c3cd7f00176467c8bea4`
- `src/chess_robot_arm/src/node.cpp`: `10d48c5c3f9e1c9d6f300640323ef2b80ec2e0ac8b05e94ad86aa712fe57c585`
- `src/chess_robot_arm/test/test_arm.cpp`: `cf35a75b6d5f13c06050ef0edbc0ad9ad0fe3d037ccc4b5cedaaccf334856497`
- `src/chess_robot_arm/test/restore_reference_offline.cpp`: `80f33981be379ac4dec206e5a5d7c634ddfe5ada9de6451a4785f56bae2c9605`

## Deployment handoff (not executed)

1. Independent reviewer validates the scoped diff and evidence and rechecks source
   hashes against `source-and-binary-sha256.json`.
2. Keep the accepted production binary available for rollback. Run the reviewed
   isolated absolute binary only after the parent approves the process change;
   source the existing ROS/underlay dependencies first. Do not overwrite the
   production install as a side effect of verification.
3. For this explicitly attested power-only interval, use the existing validated
   live configuration plus `reference.power_cycle_handoff=true` and
   `reference.power_cycle_endpoint_tolerance_rad=0.005`. Keep the existing old
   `reference.power_cycle_join_file` and original 14 audit paths/bytes, freshness,
   dwell, Home, twist and other guards. Keep enable-after-restore false for the
   initial restore check unless separately authorized.
4. Require successful detail showing the measured endpoint deltas, actual expanded
   handoff usage and retained/advanced twist. Check fresh stationary live feedback
   and original hard limits. No reference initialization/reset is needed or implied.
5. Any future Home or camera-distance motion remains a separate live planning,
   collision/path and execution check. This change authorizes no motion by itself.
