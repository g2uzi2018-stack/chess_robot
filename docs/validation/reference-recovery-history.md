# 历史参考恢复：契约、变更与证据

## 1 适用范围

1. **日常入口已更新**
   - 停稳存档取代每次启动重放大日志；[当前操作](../operations/arm-quickstart.md)。
   - 本页保留历史迁移参数和已发生结果；旧“待部署”流程已被后续部署替代，不能拿旧候选重装当前节点。
2. **恢复总约束**
   - 保留原始字节/时间戳，从已知静止Home推进每个真实关节观测，保存完整相位，最后与新鲜停稳实机端点衔接。
   - 真实无未观测运动的声明不能由端点接近代替；不清零扭转、不插值伪造历史、不扩大原记录预算。
   - 原始报告全文及未提交文档字节在 `runs/docs_cleanup_20261031_01/originals/`；路径/SHA256与迁移理由在同目录manifest，运行原证据未移动。

## 2 2026-10-29 三阶段结果

1. **有界最终端点衔接**
   - 实测最大关节差 **0.00269977955507 rad**；默认0.001拒绝，明确0.005接受，硬上限min(0.01,Home容差,关节步长)。
   - 仅声明的近Home断电最终衔接适用；普通/非Home及旧fault边界仍0.001。
   - 75项arm测试、4个localhost服务场景通过；14份原audit哈希不变。
   - total twist：0.00000578368858689→0.000716014413777rad；net：−0.00000280427711824→0.000694917184516rad，未清零。
   - 证据：`runs/power_handoff_20261029/` 的 `source-and-binary-sha256.json`、`node-loopback-results.json`、`real-history-offline-results.json`、`implementation.patch`、测试/构建日志与复现脚本。
   - 冻结节点SHA256：`25b13dccc17a24e08ee920edf0d2b28dcebc75ef0cd38de46e8a3076f84ccaf1`。
2. **记录中的停稳断电边界**
   - `handeye_live_20261029_lower01`第一帧mode4；跨第14→15文件关节差 **0.00270027889777rad**。必须声明边界后推进其后真实运动。
   - 86项arm测试、6个localhost服务场景通过；15文件前缀重放默认拒绝，声明0.005接受，完整局部相位与独立Tracker重放完全一致。
   - 净扭转 **0.0172819951221963rad**，总扭转 **0.110277324023872rad**；末端FK `[0.0303560455637507,-0.576518516138855,0.194662413994072]m`。
   - 离线前缀 `[0,229797600)` 共158700条，每条1448字节；SHA256 `e8b9c3e9329af4500ac626664bc7ccd3bf3b14113a401218e97c0f8fe8daaea4`，不替代完整现场audit。
   - 证据：`runs/power_handoff_recorded_20261029/` 的 `active-session-prefix-manifest.json`、`source-and-binary-sha256.json`、`real-history-offline-results.json`、`node-loopback-results.json`及patch/日志。
   - 旧候选SHA256 `db7f700f0234cef2c98f5b5d2d350cf0f1e448a21223c1258f69a95cf75a646f` **已被下一项修复替代**。
3. **dwell连续性修复（revision2）**
   - 旧版会把8ms静止＋992ms空窗＋孤立帧、或仅首尾stamp有效且间隔488ms的重复填充错误计入400ms停稳。
   - 现在只按通过序列检查的观测累计，原host时间不前进或间隔超过当时freshness=200ms即重开窗口；重复帧不增加dwell。
   - 正常文件切分但间隔连续可延续窗口；命名断电边界先验证断电前已完成dwell，不把断电空窗算作停稳。
   - **89个实际GoogleTest用例**通过；colcon的90条包含1个CTest包装，非新增行为。4个定向场景与15文件重放通过。
   - 真实连续停稳 **0.620084227s**≥0.4s；净/总及完整相位与上一阶段有效结果一致。
   - 证据：`runs/power_handoff_recorded_20261029/revision2/` 的 `manifest.json`、`dwell-fix.patch`、`real-history-offline-results.json`及原始失败/复测日志。
   - 节点SHA256 `406d03bfd243bf2963886f8ef154272389c713cadf518c7a834002da5184b63d`；static core `e6c8a9714c87c5007ea74316b34f519bbd272109855642749c8fe38a8507c8ff`。
   - 增量patch SHA256 `242350b60f30fc7d81a71d2d67e20fec152248d8f5df60d47ff8ad0497b32f4a`；反向和scratch应用核对通过。

## 3 后续证据

1. **冷启动fault后缀**
   - [2026-10-30的窄范围SHA绑定恢复](power-fault-recovery-20261030.md)保留完整原验收；不成为通用任意故障恢复。
2. **停稳存档与审查**
   - [2026-10-31存档报告](checkpoint-recovery-20261031.md)保留135→142项分阶段测试、迁移与实际空闲进程重启。
   - [最新1秒反馈报告](feedback-window-1s-20261031.md)为147项测试；它没有把历史100/200ms运动升级为1秒资格。

## 4 历史参数契约

### 4.1 Final live handoff

Explicit `reference.power_cycle_handoff=true` preserves the complete recorded winding
history across a declared power-only epoch after the original join/bounds/step/twist
checks. The handoff and enable-after-history-restore flags default false. Only when `controller.enable_after_history_restore=true`
is also explicitly set, a successful RestoreReference callback may check official
GetErrorID for empty alarms and call official EnableRobot for mode4; it never calls
ClearError. Enable ACK uses its own default 5 s budget. The client then requires
reference_valid, fresh mode5 and the original stationary dwell before low HOME.
No automatic reference initialization/reset or fault trajectory replay is introduced.

`reference.power_cycle_endpoint_tolerance_rad` defaults to `0.001` rad. After an
operator attests a power-only interval with no unobserved motion, an explicit
`reference.power_cycle_handoff=true` may use a separately chosen small endpoint
bound (for example `0.005` rad) for the final live join. It must be finite,
positive, and no greater than `min(0.01 rad, reference.home_tolerance_rad,
reference.max_joint_step_rad)`. The 0.01 rad (0.573 degree) cap keeps this exception
smaller than the default Home envelope and tracker step; it is not an encoder
accuracy specification. Both endpoints must still be near configured Home, with
fresh stationary live mode 4/5 dwell and the existing stopped/fault history checks.
Ordinary live joins, undeclared recorded file joins and old fault-epoch joins
remain fixed at 0.001 rad; the named stopped boundary described below also uses
the configured bounded tolerance.
This does not alter Home, motion arrival/tracking/collision limits or clear twist:
original history is replayed, then the tracker advances once to measured live
joints and checks the original hard twist limits. RestoreReference `detail` and
state diagnostics report per-joint live-minus-history deltas, the effective bound,
whether an expanded join was actually used, and historical/restored twist.
An increased bound only bounds endpoint disagreement; it cannot establish that
unobserved movement did not occur. The operator attestation remains mandatory.


### 4.2 Recorded stopped power-cycle boundary

`reference.power_cycle_stopped_join_file` defaults empty. It declares exactly one
noninitial original audit whose first disabled/idle frame records an attested
power-only handoff from stationary mode 5 Home. It requires explicit
`reference.power_cycle_handoff=true`, stationary recorded mode-5 dwell before
the boundary, mode 4/5 and stationary speed after it, both endpoints within the
original Home envelope, advancing original host time, valid joints/frames, and
maximum joint delta within the validated `power_cycle_endpoint_tolerance_rad`.
Only this declared boundary may reset its controller timestamp sequence, and the
original first frame advances the existing tracker; all later motion is replayed.
The separate `reference.power_cycle_join_file` still handles the original mode-9
fault epoch at fixed 0.001 rad. Both declarations must identify different files.
All undeclared boundaries retain the original join rules. Restore diagnostics
report the recorded joint deltas/bound and report committed/expanded usage only
after the complete history and final live join pass. No raw audit is rewritten.

Recorded mode-5 dwell counts only strictly advancing accepted host observations
whose adjacent interval is at most `feedback.freshness_s`. Ordinary file fragments
may share a window when that continuity is present. An ordinary boundary gap or
duplicate-controller-stamp padding cannot add stationary dwell: the next accepted
observation after a larger/nonforward host interval starts a new window. The named
power boundary checks the completed pre-gap window before resetting its epoch.

### 4.3 SHA-bound final fault suffix recovery

`reference.power_cycle_fault_restart_file`, `_offset_bytes` and `_sha256`
default to empty/-1/empty. Together with explicit power-cycle handoff they declare
one exact restart boundary in the unique final original audit. A healthy measured
stationary dwell must precede a short mode-9 suffix; its joints remain within the
fixed 0.005 rad healthy anchor bound. The declared fresh disabled restart and its
remaining suffix must stay within 0.001 rad. Original full-file SHA-256 must match
before committing the trial tracker. Existing history, budgets and hard twist
limits are retained; no reference zeroing or generic fault recovery is provided.

The noninstalled `restore_reference_offline` tool replays files and in-memory
feedback for regression only; it never connects to the controller. See
[the narrow recovery acceptance](power-fault-recovery-20261030.md)
for record format and historical evidence. A new power interval always needs its
own factual no-unobserved-motion declaration and valid live state; these documents
are not reusable permission to recover another run.
