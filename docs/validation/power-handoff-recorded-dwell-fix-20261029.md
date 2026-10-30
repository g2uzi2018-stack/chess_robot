# Recorded stopped handoff dwell continuity fix — revision 2, 2026-10-29

Historical implementation-stage snapshot. Later field/release evidence is indexed
in [the 2026-10-30 report](handeye-20261030.md); pending statuses below describe
this revision at its original freeze time. They are not current recovery permission.

Status: implementation frozen, affected offline checks passed, awaiting independent
recheck. No deployment, physical controller connection, robot command or service
operation was performed. The old db7f binary and all previous reports/evidence
remain unchanged. This revision corrects the blocking dwell finding in that binary.

## Defect and correction

Independent review demonstrated two actual false acceptances by the previous
frozen library: (1) 8 ms of mode-5 stationary observations followed by an ordinary
file boundary with a 992 ms gap and one isolated frame; (2) a single file containing
62 raw frames at 8 ms spacing but only its first/last controller stamps accepted,
488 ms apart. Both incorrectly supplied the required 400 ms recorded dwell even
though freshness was 200 ms. The old window accumulated elapsed host time without
checking continuity between accepted observations.

The window now resets immediately after a frame passes `FeedbackSequence::accept`
if its original host time does not strictly advance from the previous accepted
frame, or if that accepted-host interval exceeds `feedback.freshness_s`. The current
stationary mode-5 frame then starts a new window under the existing speed/anchor
conditions. Duplicate controller stamps never refresh or extend this window.
Raw host progression/freshness checks inside each audit remain separate and intact.

As agreed with the independent reviewer, ordinary file segmentation alone does
not reset the window: adjacent accepted frames may cross a normal boundary when
host time strictly advances with interval <= freshness and all stationary checks
continue. Equality at the existing freshness bound is accepted. A larger or
nonforward interval starts a new dwell window; elapsed unobserved time never counts.

The explicit stopped power boundary first checks the already completed pre-gap
dwell, then starts its declared epoch/window as before. Its attested power-off
gap is neither counted toward that dwell nor allowed to erase genuine pre-gap
proof. The old fault epoch, fixed ordinary file joins, 0.001/0.005 bounded endpoint
policy, joint/phase/twist gates, original audit bytes/times, final live join and
tracker commit behavior are unchanged. No winding or Home reference is reset.

Diagnostics additionally report `recorded_stopped_join_stationary_dwell_s`,
`recorded_stopped_join_required_dwell_s` and `accepted_host_freshness_s`. Failed
recovery continues to report committed/expanded usage false.

Only three source files differ from db7f: `src/chess_robot_arm/src/core.cpp`,
`test/test_arm.cpp` and README. No interface, node parameter, handeye, camera,
overlay, Home, geometry or motion configuration changed in this revision.

## Validation

- Isolated Release build succeeded with zero compiler warnings.
- Four new focused cases passed: 992 ms ordinary boundary gap rejects; duplicate
  controller-stamp padding with a 488 ms accepted gap rejects; genuinely contiguous
  488 ms dwell split across ordinary files accepts; exactly 200 ms accepted gaps
  with freshness 200 ms and total dwell 400 ms accept.
- Full affected arm suite: **89 actual GoogleTest cases across 19 suites**, with
  zero failures/errors/skips, including all 85 preceding cases and four new
  regressions. Colcon reports 90 records because it additionally counts one
  CTest wrapper; the wrapper is not an extra behavioral case. Existing join/fault/twist,
  exact failure-state preservation and genuine motion-history checks still pass.
- The existing real 15-file prefix replay was rerun using the new library: missing
  stopped declaration rejects, default 0.001 rad rejects, explicit 0.005 rad accepts.
  Genuine continuous accepted dwell before the recorded stopped boundary is
  **0.620084227 s**, above the unchanged 0.4 s requirement at freshness 0.2 s.
- All 14 original audit hashes, the exact original session-prefix hash and the
  active source prefix still match. No old audit, old test failure evidence or
  previous build/install/report was overwritten.
- Real-history net twist remains **0.0172819951221963 rad**, total twist
  **0.110277324023872 rad**; FK is
  **(0.0303560455637507, -0.576518516138855, 0.194662413994072) m**.
  Complete independent direct Tracker replay still agrees with the restored
  local winding vector, metrics and final q; no zeroing occurred.

No unrelated handeye/camera tests or live ROS/robot sessions were rerun for this
internal dwell correction. The parent/reviewer will perform independent recheck
against the new frozen static core, preserving their original failing evidence.

## Reproduction

From `/home/kuang/workspace/chess_robot`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon --log-base runs/power_handoff_recorded_20261029/revision2/log build \
  --base-paths src/chess_robot_arm --packages-select chess_robot_arm \
  --build-base runs/power_handoff_recorded_20261029/revision2/build \
  --install-base runs/power_handoff_recorded_20261029/revision2/install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  --event-handlers console_direct+
colcon --log-base runs/power_handoff_recorded_20261029/revision2/log test \
  --base-paths src/chess_robot_arm --packages-select chess_robot_arm \
  --build-base runs/power_handoff_recorded_20261029/revision2/build \
  --install-base runs/power_handoff_recorded_20261029/revision2/install \
  --event-handlers console_direct+
colcon test-result --test-result-base runs/power_handoff_recorded_20261029/revision2/build/chess_robot_arm --verbose
python3 runs/power_handoff_recorded_20261029/revision2/real-history-offline.py
```

## Frozen identities and artifacts

New absolute node:
`/home/kuang/workspace/chess_robot/runs/power_handoff_recorded_20261029/revision2/install/chess_robot_arm/lib/chess_robot_arm/arm_node`

Node SHA256:
`406d03bfd243bf2963886f8ef154272389c713cadf518c7a834002da5184b63d`

New static core:
`revision2/build/chess_robot_arm/libchess_robot_arm_core.a`
SHA256: `e6c8a9714c87c5007ea74316b34f519bbd272109855642749c8fe38a8507c8ff`.

Old recorded-boundary node remains
`db7f700f0234cef2c98f5b5d2d350cf0f1e448a21223c1258f69a95cf75a646f`;
old live-endpoint node remains
`25b13dccc17a24e08ee920edf0d2b28dcebc75ef0cd38de46e8a3076f84ccaf1`;
original production install remains
`1b5cf1d508bd49afc9b9608c3503940c12da7742a935146945d9d3c1fa848443`.

Within `runs/power_handoff_recorded_20261029/revision2/`:

- `manifest.json`: exact source/binary hashes and source scope.
- `dwell-fix.patch`: three-file incremental correction against the SHA-verified
  reconstructed db7f source. Patch SHA256:
  `242350b60f30fc7d81a71d2d67e20fec152248d8f5df60d47ff8ad0497b32f4a`.
  Reverse apply-check and actual scratch application both passed; all resulting
  source hashes equal the frozen source manifest.
- `implementation.patch`: full package diff against repository HEAD plus new files.
- `build.log`, `dwell-tests.log/xml`, `all-tests.log`, `test-result.log`.
- `real-history-offline.py`, `real-history-offline-results.json`, console log;
  these read the unchanged original evidence but write results only in revision2.

## Parent handoff

Use the new **revision2** absolute binary/hash in the pending helper after independent
recheck; do not deploy the superseded db7f candidate. The prior report's controlled
stop/fix-complete-audit/replay/restore procedure otherwise remains applicable.
The full currently appending real session must still be fixed and validated after
the parent safely stops its owned idle arm node; the saved offline prefix is not
a replacement for that complete session. Keep both explicit epoch declarations,
all original gates, a new audit output directory, and the intended general-point
`testing.home_roundtrip_precheck=false` setting. Enable remains a separate parent
decision. Existing 16-file and two-million-frame replay budgets are unchanged.
This implementation work performs none of those deployment or motion steps.
