# Recorded stopped power-cycle join — 2026-10-29

Historical implementation-stage snapshot. Later field/release evidence is indexed
in [the 2026-10-30 report](handeye-20261030.md); pending statuses below describe
this revision at its original freeze time. They are not current recovery permission.

Implementation and offline validation complete; deployment pending independent
review. This is a second work package building on the accepted live endpoint
recovery. The previous report, evidence directory, install, original production
install, original audits, running robot node and preview services were not changed.
No physical controller connections or commands were made in this work package.

## Why this boundary must be recorded

The previous binary successfully restored the original 14-file history across an
explicitly attested power-only restart using a 0.005 rad live endpoint bound.
The new real session then recorded Enable, Home, an 80 mm base-Z descent and idle.
Its original first frame is disabled mode 4, with a 0.00270027889777 rad maximum
joint difference from file 14. A process restart must replay this actual boundary
and all subsequent measured motion. The ordinary fixed 0.001 rad / mode-5 file
join deliberately refuses it. Ignoring this session or initializing at Home would
lose the measured winding history.

## New bounded declaration

`reference.power_cycle_stopped_join_file` defaults empty. When set, it must name
exactly one noninitial audit in the request, distinct from the existing
`reference.power_cycle_join_file`, with `reference.power_cycle_handoff=true`.
Only this declared boundary permits the following recorded stopped handoff:

- Previous accepted frame: mode 5, speed within the existing stationary threshold,
  with contiguous recorded mode-5 stationary dwell using the existing dwell time.
- First actual frame of the declared file: mode 4 or 5, valid framing/finite fields,
  valid model joint bounds and stationary speed.
- Both joint endpoints remain within the original configured Home envelope;
  original host time strictly advances across the boundary.
- Maximum joint disagreement is at most the existing validated
  `reference.power_cycle_endpoint_tolerance_rad` (default 0.001; explicit 0.005
  for this attested interval; hard cap remains min(0.01, Home tolerance, tracker step)).

Only after those checks may that named file start a new recorded controller clock
sequence. Its real first q advances the existing tracker. Every later original
frame, motion, joint/phase step, history freshness/mode and twist limit is checked.
No interpolation, timestamp rewriting, record replacement or reference zeroing.
The separate old mode-9 fault epoch declaration keeps its original fixed 0.001 rad
bound and once-only semantics. All undeclared boundaries keep the original rules.
The final live join is unchanged; at the lowered location it uses the original
0.001 rad ordinary bound, not the expanded near-Home power-cycle bound.

The existing RestoreReference schema is unchanged. Detail reports recorded
per-joint deltas, effective bound, `recorded_stopped_join_committed` and
`recorded_expanded_stopped_join_used`. Both usage flags are false on failed
restoration, including failures later in the replay/final handoff; no trial state
is committed on rejection. The existing accumulated twist diagnostic is retained.

Source changes relative to the preceding accepted package are limited to seven
arm files: Config header, core, ROS parameter declaration, default YAML, README,
unit tests and the test-only offline tool. CMake is unchanged from that package.
No public ROS messages, handeye/vision/preview package or canonical Home changed.

## Original active-audit prefix evidence

Only a read-only complete-record prefix was copied from:
`runs/handeye_live_20261029_lower01/audit/feedback.bin`.

- Snapshot: `runs/power_handoff_recorded_20261029/active-session-original-prefix.bin`
- Original byte range: **[0, 229797600)**, 158700 records of exactly 1448 bytes.
- SHA256: `e8b9c3e9329af4500ac626664bc7ccd3bf3b14113a401218e97c0f8fe8daaea4`.
- Original source prefix was reread and compared byte-for-byte at capture and
  SHA-checked again after replay; it remained identical while the source appended.
- First frame: true mode 4, host 1793260260962663513,
  controller stamp 1793262502238.
- Last captured frame: true stationary mode 5, host 1793261558980014366,
  controller stamp 1793263800270.

`active-session-prefix-manifest.json` records the source path, byte interval,
record count, hash, and original first/last joints/speeds. No original raw file
was written. This prefix is offline evidence only; deployment must use the full
fixed original session audit after the parent safely stops its idle node.

The other 14 complete files come from the SHA-bound `audits` list in the existing
`runs/handeye_live_20261029_lower01/result.json`. The old fault epoch remains
`runs/power_recovery_20261026/physical_continue01/audit/feedback.bin`. This session's
parameters, source binary and recorded restore detail bind the no-unobserved-motion
operator statement to the expanded small near-Home handoff.

## Validation results

- Release build: success, zero compiler warnings.
- Entire affected arm suite: **86 tests, 0 errors/failures/skips**: all 75 previous
  cases plus 11 new recorded-boundary cases.
- New tests cover default/invalid/duplicate/initial/conflicting declarations;
  undeclared or strict-default rejection; old fault epoch + new stopped epoch
  composition; unchanged old fault/other file bounds; stationary dwell/near-Home
  requirements; invalid mode/speed/joints/host/controller time; excessive delta;
  post-boundary joint-step/twist guards; full subsequent motion replay;
  required attestation/final join; and failed replay preserving an existing
  valid reference's exact nonzero metrics and last joints.
- Localhost ROS integration: **6 service cases passed**, with simulated state
  reference/twist checked. Domain 79, localhost-only, test_fixture, paused
  read-only backend, Enable-after-restore false; zero dashboard commands and
  zero physical connections. Only its own three test nodes were terminated.
- Real **15-file** replay: undeclared boundary rejects; declared default 0.001 rad
  rejects; declared 0.005 rad accepts. All 14 original manifest SHA256 values,
  the copied prefix SHA256 and the active source prefix remained unchanged.
- Accepted recorded boundary maximum delta: **0.00270027889777 rad**.
- Restored net twist: **0.0172819951221963 rad**.
- Restored total twist: **0.110277324023872 rad**.
- FK of the last original measured q:
  **(0.0303560455637507, -0.576518516138855, 0.194662413994072) m**.
- The independent direct Tracker replay consumed every accepted original frame
  across both named epochs and compared net, total, energy, the entire local
  winding vector and final joints with State::restore: **exact agreement**.

The offline utility repeats only labelled in-memory test endpoint snapshots to
keep the live side fresh during a long replay, using the original last q/mode
and an explicitly synthetic continuing controller stamp. It never appends these
test snapshots to any real audit. Its original host/record bytes remain untouched.

## Reproduction and evidence

From `/home/kuang/workspace/chess_robot`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon --log-base runs/power_handoff_recorded_20261029/log build \
  --base-paths src/chess_robot_arm --packages-select chess_robot_arm \
  --build-base runs/power_handoff_recorded_20261029/build \
  --install-base runs/power_handoff_recorded_20261029/install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  --event-handlers console_direct+
colcon --log-base runs/power_handoff_recorded_20261029/log test \
  --base-paths src/chess_robot_arm --packages-select chess_robot_arm \
  --build-base runs/power_handoff_recorded_20261029/build \
  --install-base runs/power_handoff_recorded_20261029/install \
  --event-handlers console_direct+
colcon test-result --test-result-base runs/power_handoff_recorded_20261029/build/chess_robot_arm --verbose
python3 runs/power_handoff_recorded_20261029/node-loopback.py
python3 runs/power_handoff_recorded_20261029/real-history-offline.py
```

Base paths intentionally exclude unrelated cached Python environments under runs.
The initial unit fixture success cases incorrectly used a synthetic live clock
older than the recorded last clock; correcting the fixture to a newer clock made
them pass while retaining the original final-clock protection.

Evidence in `runs/power_handoff_recorded_20261029/`: `final-build.log`,
`all-tests.log`, `test-result.log`, `node-loopback-results.json`,
`real-history-offline-results.json`, `active-session-prefix-manifest.json`,
`source-and-binary-sha256.json`, `incremental-source.patch`, `implementation.patch`,
and both reproducible harnesses. The incremental patch compares against the
previous accepted source reconstructed from its original patch and verified
against every previous source SHA256 before comparison.

## Source and binary identity

New isolated node SHA256: `db7f700f0234cef2c98f5b5d2d350cf0f1e448a21223c1258f69a95cf75a646f`.

Running previous isolated node remains
`25b13dccc17a24e08ee920edf0d2b28dcebc75ef0cd38de46e8a3076f84ccaf1`.
Original production node remains
`1b5cf1d508bd49afc9b9608c3503940c12da7742a935146945d9d3c1fa848443`.
The previous validation report/patch/source manifest hashes are captured without
modifying them in the new identity manifest.

Source SHA256:

- `src/chess_robot_arm/CMakeLists.txt`: `b121f6a90b38e0bf9f2c01a9c5e4985eac2a68b625087a1dbc1dacc381b7ef60`
- `src/chess_robot_arm/README.md`: `c006b33d88f08b065f80a059ac082603bacdf13bf1b617ecea526102dc279912`
- `src/chess_robot_arm/config/arm.yaml`: `dc967c7cfece9ff2e9828527c58f49f42a6d9c51c42954b2e70afb64fda706d7`
- `src/chess_robot_arm/include/chess_robot_arm/core.hpp`: `011d8e0c65adb7176b865c7bc17b1514c09acc58c8fc416ccd7bb65823d819ee`
- `src/chess_robot_arm/src/core.cpp`: `414911f021080ac0f98341bb2db833b6901626987cb3c4c5ea53d01a1ae2c49e`
- `src/chess_robot_arm/src/node.cpp`: `71a0ea1fb20db0300b23200c2adcd3a8da573c16936c54024ad32a25030d738d`
- `src/chess_robot_arm/test/test_arm.cpp`: `1f2e676a506d5b9860a0824f524a29babb016f873ebd3ce341d414ebbc54f6b5`
- `src/chess_robot_arm/test/restore_reference_offline.cpp`: `3bc4aaa98a0005ee276106a25273f17ad8e0b4e8c4c49b09d740d59dd0452833`

## Parent-controlled switch procedure (not executed here)

1. Complete independent review of the incremental diff, unchanged prior artifact
   hashes, new binary hash and offline evidence. Keep the previous binary intact.
2. Confirm the existing real node is idle with fresh feedback. The parent may
   then safely stop **only its owned idle arm node** and wait for its audit to
   finish flushing. Do not power-cycle the controller, move joints, initialize a
   reference, clear errors, or stop camera/preview services for this config switch.
3. Fix the **complete original** `handeye_live_20261029_lower01/audit/feedback.bin`
   as immutable replay evidence; record size/full SHA256 and require size modulo
   1448 = 0. Verify its first 229797600 bytes still have the captured prefix hash.
   A truncated tail must be investigated, not silently truncated or rewritten.
   The offline prefix does not replace this complete real audit for deployment.
4. Prepare a new runtime parameter file preserving the original Home, TCP,
   geometry, speed/profiles, freshness 0.2, stationary threshold 0.002, dwell 0.4,
   motion/tracking/arrival/twist gates. Set only the intended general-point mode
   `testing.home_roundtrip_precheck=false`, the two explicit join declarations,
   and a **new** audit output directory for the next session. Keep:
   `reference.power_cycle_handoff=true`, endpoint tolerance 0.005,
   old `reference.power_cycle_join_file` as above, and new
   `reference.power_cycle_stopped_join_file` pointing to the fixed complete
   `handeye_live_20261029_lower01/audit/feedback.bin`. Enable-after-restore is a
   separate explicit parent decision; this implementation does not require or
   initiate it. Current mode 5 should not need EnableRobot for a process restart.
5. Re-run the offline 15-file check against that full fixed file and its last
   recorded endpoint before live restoration. The reviewed test-only CLI is:
   `restore_reference_offline URDF TOLERANCE_RAD FAULT_EPOCH_JOIN_FILE STOPPED_JOIN_FILE LIVE_Q_MODE_STAMP_FILE AUDIT...`.
   It needs six radian joints, mode and continuing test clock in the snapshot file.
6. Start the new reviewed **absolute isolated binary**
   `runs/power_handoff_recorded_20261029/install/chess_robot_arm/lib/chess_robot_arm/arm_node`
   with original Home and the new runtime parameters. Do not overwrite either
   prior install. Require fresh stationary feedback, then call the existing
   RestoreReference with the original 14 complete paths plus the fixed 15th
   complete original session file and the appropriate attestation.
7. Require successful detail with both expected epoch declarations,
   `recorded_stopped_join_committed=true`, expanded recorded usage true,
   retained nonzero twist near 0.11027 rad, and the ordinary final live bound
   0.001 rad. Check fresh idle state/FK and original hard gates. If restoration
   refuses, retain all evidence and resolve the stated discrepancy; do not zero.
8. Subsequent general-point planning/handeye work is separate and parent-owned.
   This package executes no motions. The existing 16-file / two-million-frame
   replay budgets remain unchanged; the 15-file history fits those bounds.
