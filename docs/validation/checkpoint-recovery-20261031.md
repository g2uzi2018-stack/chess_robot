# Stopped checkpoint recovery and bounded diagnostics — 2026-10-31

本页保留2026-10-31批次及明确标注的后续修复证据；“当前/未加载/回退”均指记录阶段。后续Home目标与历史基准已分开，旧部署/回退命令仅适用原批次，当前入口见[日常操作](../operations/arm-quickstart.md)。不要用旧报告覆盖新安装或改写旧测试结果。

Implemented in the existing C++ arm node. No separate motion controller, reference
zeroing, relaxed geometry/arrival/stop limits or production TF update was introduced.
The operator explicitly promises no movement while powered down, and powers down
only after stopping. This persistent policy is an installation opt-in, not something
endpoint matching alone can prove.

## Runtime behavior

- Save every 10 s while healthy/stopped, and promptly after 1 s stopped dwell at task
  completion or observed motion. A valid mode5 reference and idle/non-faulted task
  admission are required. Writes stay off the feedback thread.
- Persist complete local unwrapped phases and measured joints, not just net/total.
  Schema/model/Home/TCP/controller fingerprint and SHA256 detect incompatible or
  damaged files. Atomic rename follows file fsync and precedes directory fsync;
  a single-writer lock prevents competing checkpoint owners.
- Durably mark the save dirty before any actual motion send. Interrupted motion,
  dirty/missing/corrupt/incompatible checkpoints and unmatched endpoints do not
  restore or fall back to an older save. Returning Home never initializes zero twist.
- Restore at a matching stopped non-Home mode4/5 endpoint with the operator policy,
  fresh continuous dwell, joint bounds and current twist limits. Default endpoint
  allowance is 0.005 rad, capped at 0.01 rad and the original joint-step limit.
- After successful restoration, optionally GetErrorID then EnableRobot. No ClearError
  and no automatic HOME/other movement. Enable failure invalidates tentative trust
  and inhibits repeated automatic enable attempts for that process.
- A disabled restarted telemetry clock can be admitted without granting reference;
  complete checkpoint validation still precedes enable/motion.
- Diagnostic raw/command/ServoJ logs use bounded asynchronous queues and whole-record
  rotation. Default 64 MiB and 16 files per stream include the active segment.
  Existing new-format log directories survive restarts and rotate previous active
  files. Queue/disk errors are visible in state diagnostics, not motion references.
  Historical pre-change audit directories were retained unchanged.

Installation state is `.runtime/arm-checkpoint.save`, configuration `.runtime/arm.yaml`,
and logs `.runtime/audit/`. These are ignored by Git. Launch with
`bash tools/arm/run_with_checkpoint.sh`; `.runtime/arm-install-prefix` selects the
installed release, falling back to the workspace `install` prefix when absent.
The launcher itself does not initialize reference, clear alarms or command motion.

## First migration and verification

The original 16 SHA-bound audits passed unchanged legacy restoration. A separately
SHA-bound 6,527,444,992-byte continuation prefix was then streamed once offline,
including all accepted motion and the sole stopped power boundary. Source records
were neither trimmed nor rewritten; old restoration budgets were not raised.
4,507,437 continuation frames were accepted under the normal duplicate rule.
The 26.6 ms tail after the last confirmed stopped observation included brake/power
settling; the maximum fault displacement was 0.0029390313 rad. Every recorded q was
advanced; the complete suffix result agreed with joining the saved stopped state
directly to the final bounded disabled endpoint within 1e-8 rad per local phase.
This generated the first clean checkpoint; future routine recovery does not read
these source audit files.

135 unit tests passed: 99 arm, 20 twist and 16 checkpoint/logging tests. Additional
loopback ROS integration verified two clean node starts followed by plan-only HOME,
and dirty-checkpoint rejection; no physical controller was connected by those tests.
The installed physical node restored a nonzero reference, checked empty alarms and
enabled. A real idle process restart then preserved signed/total twist within 1e-4
rad, kept the measured pose, rotated previous raw logs and produced another durable
save approximately 10.13 s after the preceding save. These were process restart and
the already recorded real power boundary, not an additional commanded power-cycle test.

All evidence is under `runs/checkpoint_recovery_20261031_01/`: migration plan/log,
source hashes, unit XML/logs, loopback results, deployment, live-recovery, live-restart,
finite action results and final summary. The installation remains in the existing
bounded exploration scope; no new hardware certification claim is made.

## Physical HOME and laser observation point

Three actual goals through `/arm/move` completed with measured stationary dwell:
HOME; a separately reviewed return waypoint above the original start height; CH60.
HOME planning and execution passed after checkpoint recovery. The return waypoint
was 5 mm above the original measured start pose, with sampled minimum TCP height
77.283 mm versus the original 72.610 mm. The corner move used profile `alignment`
with 0.4 rad/s and 0.4 rad/s² planning caps, compared with the existing 0.3 low cap.
Its target included 1 mm extra Z clearance to keep approximate IK from dipping below
that lateral move's starting height. This is clearance, not calibration compensation.

CH60 was re-observed immediately before each goal with the actual image stamp;
61 corners and 0.2245 px reprojection RMS were recorded before execution. Its fresh
position agreed with the reviewed target within 0.5 mm. Final measured model TCP
height was approximately 77.96 mm, still above the original start. No gripper command
or additional descent was issued. All three trajectories were reviewed before an
actual goal, and feedback/corridor monitoring retained owned cancellation and stopped
proof on failures. No automatic retry of an actual motion occurred.

The target corner is occluded by the added tool in the final camera view. The image
preview labels its board-pose projection as `projected (occluded)`; it does not claim
to observe the corner or measure the laser spot there. The operator's laser error
reading, beam alignment and physical millimetre accuracy remain unquantified.


## Independent review corrections (2026-10-31)

The subsequent independent source review found and the primary reproduced three
defects: reference loss during GetErrorID could still send EnableRobot; PID-named
interrupted temp files could block future writes; reduced retention left higher
numbered archives behind. They are fixed with a generation-bound live guard under
command ownership before enable plus a post-ACK validation, exclusive randomized
temporary names, and canonical archive pruning on startup. No guards were relaxed.

142 unit tests (99 arm, 23 persistence/logging, 20 twist), three loopback ROS recovery
scenarios and before/after C++ reproductions passed. Incremental independent source
review found no remaining issues in these fixes. The peer CLI attempt exceeded its
budget without a valid handoff and was not counted as a passed review; a separate
built-in subagent completed the independent review. Full records are in
`runs/checkpoint_review_20261031_01/`.

The repaired release was loaded by restarting only the verified idle enabled node.
Reference, winding and measured pose were preserved. Neither old nor new command
logs recorded any SDK command during this reload. No hardware motion or additional
physical power-cycle test was performed in the review task.


## 2026-11-01 后续修订

以上保留 10 月 31 日实现/验收记录；其中 mode5-only 保存与“任何 enable 失败都 invalidate”已被 [历史与视觉解耦修复](history-vision-decouple-20261101.md) 替代。新实现允许停稳 mode4 保存，历史与执行器资格分离，并按新反馈确认决定保留或失效。旧验收不能当作新二进制已加载实体的证明。
