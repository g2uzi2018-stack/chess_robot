# SHA-bound disabled cold-restart recovery — 2026-10-30

Historical implementation-stage snapshot. Later field/release evidence is indexed
in [the 2026-10-30 report](handeye-20261030.md); pending statuses below describe
this revision at its original freeze time. They are not current recovery permission.

Implementation frozen; self-tests and complete original 16-audit offline replay
passed. Independent review pending. No controller/socket/service operation,
deployment, Enable/ClearError/Home/reference initialization or motion was performed
by this implementation task. Previous binaries and all raw audits remain unchanged.

## Narrow new declaration

The final audit contains a real healthy stationary non-Home position, two original
mode-9 fault samples, a 53206.255495488 s host gap, then a disabled mode-4 epoch.
The final fault sample has actual J2 velocity -0.0109056447 rad/s and a recorded
0.00320268423742 rad displacement from the healthy anchor. It is not healthy or
stationary evidence. Its measured q must advance winding history before restoration.

Three immutable ROS parameters default to empty / -1 / empty:

- `reference.power_cycle_fault_restart_file`
- `reference.power_cycle_fault_restart_offset_bytes`
- `reference.power_cycle_fault_restart_sha256`

They must be declared together with explicit `reference.power_cycle_handoff=true`
and the existing service's `no_unobserved_motion=true`. The file must occur exactly
once as the final requested audit and differ from both earlier join declarations.
The positive offset must be aligned to a complete 1448-byte original record.
A complete lowercase SHA256 binds the entire file; EVP hashes original host/frame
bytes during replay, outside the state mutex. OpenSSL::Crypto/libssl-dev is the
only new build dependency. SHA identity does not replace the operator attestation.

Before the offset, the new fault suffix must immediately follow continuous fresh
accepted mode-5 stationary dwell. Duplicate stamps cannot supply dwell. Fault
samples require advancing controller stamps, stay within a fixed 0.005 rad of the
last healthy q and within the existing freshness interval from that last healthy
observation. Their finite actual velocities may be nonzero. Every fault q advances
the original trial tracker and passes the same joint/phase/twist hard limits.
Fault samples never refresh healthy dwell or establish arrival.

Only the exact declared offset may cross the in-file host gap and reset the
controller sequence. Both the prior raw and accepted sample must be mode9; the
first new sample must be mode4, stationary, valid, after a real host gap, and within
fixed 0.001 rad of the final fault q. Every subsequent raw frame must remain mode4,
stationary and within 0.001 rad of the restart anchor; any later gap/regression is
rejected under the original rules. The declaration must actually be consumed and
the full file hash must match; neither can silently fall back to an older branch.

After complete replay, the final live state must be fresh stationary mode4 dwell,
within 0.001 rad of both the historical endpoint and restart anchor, with controller
stamp strictly newer than the last recorded accepted stamp. The old near-Home clock
exception cannot apply to this declaration. The existing hard twist limits are
checked after advancing once to live q; only then is the trial committed. Failure
leaves existing reference/metrics unchanged and never reports successful commit.

The old near-Home live, mode9 file-epoch and stopped-file joins retain their original
semantics and bounds. Home/TCP/geometry/motion gates, service schema and the existing
16-file / two-million-raw-record budgets are unchanged.

## Exact original fixture and successful result

Full final original audit:
`runs/handeye_live_20261029_normal01/audit/feedback.bin`

- 1013705704 bytes, 700073 complete records.
- SHA256 `1a219cb39e9d0eaf2e4d227f66e2f67fd3b36e7e852f20a8185051ac828dc413`.
- Restart offset **970581368**, original record index 670291.
- Pre-offset prefix SHA256
  `ef5ef4f15b90c61568fe10ca125b272dd60cc6e49e8b5b6ac599ca97e61e1352`.
- All 16 complete original files total **1817739 raw records**, below 2000000.
- The parent/live agent froze the owned disabled idle old node; this task only read
  those original files and did not truncate, replace, synthesize or append records.

New product State::restore replayed the complete original 16 files successfully.
Every source SHA256 matched before and after replay. Independent direct Tracker
replay of original accepted q, including both fault q and the exact epoch reset,
agreed with the restored local winding vector, metrics and final q.

- Healthy total twist: **0.294923837762 rad**.
- Final fault total twist after actual measured displacement: **0.295132896439 rad**.
- Final restored total: **0.295139381101961 rad**; net **-0.0503350027799895 rad**.
- Fault-to-restart maximum joint delta: **0.0000517651936884 rad**.
- FK at final original q: **(0.0225914500204529, -0.583522788383933, 0.191830402061108) m**.

The offline test snapshot is explicitly labelled and keeps only in-memory test
feedback fresh; it does not claim new physical observations or write real audits.
`offline-parameters.yaml` records the corresponding model/reference/twist settings,
with read-only loopback/motion-paused/hardware-disabled settings to avoid accidental
live use. Actual startup parameters remain parent-owned.

## Validation and frozen delivery

Release build passed without compiler warnings. **98 actual GoogleTest cases across
20 suites passed**, comprising all 89 preceding cases plus 9 new test groups. The
new groups cover defaults/incomplete/invalid declarations, absent/wrong offset/SHA,
mode5 rejection at first/post/live, nonstationary healthy/restart/post/live rejection,
nonstationary fault acceptance with true q advancement, fixed anchor/restart/live
bounds, fault duration, other gaps/clock regression/no near-Home clock bypass,
required attestation/unique final file/consumed offset, and failures preserving an
existing nonzero valid tracker. Old fault/stopped joins and winding guards regressions
remain green. No unrelated camera/handeye/service tests were added or rerun.

New node:
`runs/power_fault_recovery_20261030/install/chess_robot_arm/lib/chess_robot_arm/arm_node`
SHA256 `f6810c22212bff2e4fc481c2e489e6a937d327db4133b06aaa624a91d2ae4839`.

Core SHA256: `ac03aeed9cde5bbf65a516c718bcd5729d8579349cfa878b17ac308a46f94cde`.
Offline CLI SHA256: `418f91211908cb279bf3d5568807c5573f55066d9d93653fb98957d92ab20fe3`.
Previous 406d isolated node remains unchanged.

Evidence under `runs/power_fault_recovery_20261030/`:
`manifest.json`, `fault-restart.patch`, `audit-manifest.json`, `offline-parameters.yaml`,
`build.log`, `fault-tests.log/xml`, `all-tests.log/xml`, `full16-results.json`,
`full16.stdout`, `full16-command.json` and `full16-offline.py`. The incremental patch
was actually applied to a SHA-verified 406d source copy and produced exactly the
new frozen source hashes. No source was committed or pushed.

Reproduction after sourcing ROS Humble and the existing install dependencies:

```bash
colcon --log-base runs/power_fault_recovery_20261030/log build \
  --base-paths src/chess_robot_arm --packages-select chess_robot_arm \
  --build-base runs/power_fault_recovery_20261030/build \
  --install-base runs/power_fault_recovery_20261030/install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
runs/power_fault_recovery_20261030/build/chess_robot_arm/test_arm
python3 runs/power_fault_recovery_20261030/full16-offline.py
```

CLI remains `restore_reference_offline URDF OLD_TOLERANCE OLD_FAULT_JOIN
OLD_STOPPED_JOIN SNAPSHOT [--fault-restart FILE OFFSET SHA256] AUDIT...`.
The parent must await independent PASS, bind the exact frozen SHA/offset plus both
old join declarations and all 16 original paths, and independently check fresh
mode4 recovery before deciding Enable/Home. `testing.home_roundtrip_precheck=false`
remains the intended general-point setting. No automatic reference zeroing is needed.

This is one deliberately narrow recovery of the final disabled suffix. It is not
unlimited power-cycle history recovery. Future motion-audit appends, another restart,
extra files or approaching the raw budget require a separately reviewed design;
this change does not authorize trimming history, raising budgets or reusing a stale
attestation. Subsequent physical Home/handeye execution is entirely parent-owned.
