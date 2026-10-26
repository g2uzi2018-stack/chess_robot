# 有限 Home → 实际 TCP → Home 验收工具

入口 `tools/testing/arm_roundtrip.py`。本工具只调用既有 `/arm/move` 的 PREGRASP/HOME，
启动时显式 `approach_clearance_m=0`、`testing.home_roundtrip_precheck=true`，因此给定坐标就是
实际 TCP 终点。没有抓取、夹爪、下潜、撤离或额外朝向动作。消息、话题和 action/service 格式不变。

正式 Home 从项目原 `src/chess_robot_arm/config/home.yaml` 读取并核对另一份
`config/arm/home.yaml` 哈希，不重新定义 Home。朝向固定为正式 Home 的标称 TCP 朝向。
节点对每个 IK 候选用同一个复制的扭转历史检查去程和预计目标→正式 Home 返程；只有两段通过
才允许去程。真实返程仍是独立 HOME action，按最新反馈重新检查、执行和确认。

## 当前状态

2026-10-26：历史14个不同目标实测往返成功；断电后显式恢复历史与使能、low Home通过，
新epoch低档资格因146.428485 ms反馈接收空窗失败，业务Stop及新鲜停稳确认成功。
当前BLOCKED、实机暂停，未在Home且参考失效；新增目标成功0、边界0/12，不宣称全部完成。
设备绝对再现/缩减速度及加速度上限仍未知；本次 `--command-policy` 记录用户请求的高1/低0.3
rad/s及1/0.3 rad/s²规划上限，不是本机硬件限值认证或实测速率。
见 `docs/validation/home-roundtrip-20261026/REPORT.md`；10/25报告保持历史，模拟结果单列。

## 有限清单和时间

默认 26 点：基于正式 Home TCP，基座坐标 ±X/±Y/±Z 六方向各 0.15、0.30、0.60 m 共 18 点；
再在 Home 上下 0.15 m 两高度各测左右前后 0.30 m 四点，共 8 点。低于工作区/桌面等非法点
保存规划拒绝，不尝试运动。方向按基座轴定义，不借用相机视角。

每个方向最多 2 个边界探测槽：已有较远拒绝时二分已实测往返成功与拒绝之间的区间；若 0.60 m
也成功，先用一个槽检查配置工作区边缘，再视结果用剩余槽加密。总计最多 38 个不同目标。
若边界仍未闭合，明确报 unbracketed，不能说找到全局最远位置。达到配置工作区边缘也仅是
该配置域内的测量，不是机械臂全空间证明。

没有已有真实两档资格证据时，在同一工具中用清单内一个通过两段预检且有普通转移余量的目标
先做 low 往返、再做 high 往返，全部反馈确认才记录资格；此目标后续复用，避免重复。
含资格阶段最多 40 次往返尝试。显式初始 Home 恢复最多再加 1 段，即最多 81 段运动。
初始估计 10–25 分钟；首个目标运动前打印全部粗扫的实际节点去返规划时间总和，区分运动时间、
SDK 规划和 dwell，不把该粗估当作保证时限。故障立即结束，不自动重放或强行回 Home。

## 离线生成清单与回环

```bash
cd /home/kuang/workspace/chess_robot
runs/urdf_tools_env/bin/python tools/testing/arm_roundtrip.py \
  --make-manifest runs/home_roundtrip_20261025/manifest.json
python3 tools/testing/arm_roundtrip_test.py
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=98 ROS_LOCALHOST_ONLY=1 runs/urdf_tools_env/bin/python \
  tools/testing/arm_roundtrip_integration.py --out runs/new_roundtrip_loopback
```

回环仅连接 127.0.0.1 随机端口，输出 `/simulation/arm/*`。模拟资格文件保留
`simulation=true`，真实工具拒绝把它当作真实速度证据。正式工具本身不调用 InitializeReference；
仅回环 harness 在模拟 Home 显式建立一次模拟参考供测试。

## 真实执行前置条件与入口

当前限值未知时不要填入猜测值。`--controller-limits` 要求 JSON 含：

```json
{
  "model": "Nova5",
  "firmware_version": "4.6.2.4-stable-d9efa2ce0-202509261512",
  "source": {"kind": "实际来源类型", "reference": "实际控制器配置/型号资料的可复核来源"},
  "units": {"velocity": "rad/s", "acceleration": "rad/s2"},
  "velocity_rad_s": [null, null, null, null, null, null],
  "acceleration_rad_s2": [null, null, null, null, null, null]
}
```

这是字段说明，null 模板不是有效限值，工具会拒绝。文件不是 `verified=true` 认证开关。
实际值与官方配置逐轴取更严格值并真正写入节点 limits/profile；记录请求值和每轴生效值。
百分比不当作 rad/s。普通余量路径使用已合格 high；低高度、边界探测、桌面/自碰包络、关节或
奇异性余量不足时使用 low；缺少独立路径余量证据也不贸然用 high。没有另设极低调试速度封顶。
全批预检仅用于记录和估时；每次实际去程前重新规划并重新选档，不用缓存余量决定新路径速度。
独立 evaluator 同时检查去程和预计目标关节→正式 Home 的关节直线返程采样，后者与节点 HOME
PTP 的几何路径一致；它仍是离散包络余量，并非连续碰撞证明。

`--history-audit` 可重复传入完整、有序的原始审计文件，交由既有 RestoreReference 服务验证；
`--history-attestation` 必须保留用户对记录空窗无外部运动/拖拽/复位的明确原话，形如
`{"no_unobserved_motion": true, "user_statement": "用户的实际确认原话"}`。不得编造声明。
恢复既有参考，不调用自动初始化/清零。

```bash
ROS_DOMAIN_ID=98 ROS_LOCALHOST_ONLY=1 runs/urdf_tools_env/bin/python \
  tools/testing/arm_roundtrip.py \
  --manifest runs/home_roundtrip_20261025/manifest.json \
  --controller-ip 192.168.30.234 \
  --out runs/new_finite_roundtrip \
  --execute --recover-home \
  --controller-limits /path/to/actual-controller-limits.json \
  --history-attestation /path/to/actual-user-attestation.json \
  --history-audit /path/to/ordered-prior-audit/feedback.bin \
  --margin-evaluator runs/workspace_final_20261024/workspace_eval
```

没有当前来源限值或明确 `--command-policy` 时，execute 在连接控制器/启动节点前 PRECONDITION_BLOCKED。
`--command-policy` 仅在用户明确指令时记录原话、单位和请求参数，实际写入本次节点并保留
hardware_limits_verified=false；不可伪造控制器限值文件，也不将百分比换算成绝对rad/s。
默认要求当前已在正式 Home。仅用户显式传 `--recover-home`，且限值/真实历史恢复已通过时，
先计划一次 low HOME、再经业务节点实际执行并反馈确认；该独立恢复阶段不算目标成功。
恢复失败即结束，不重试，不重置参考。

同一进程先完成 low/high 两个实测资格往返才进入清单。生成的 `speed_qualification.json`
绑定节点二进制、C++/工具源码、Home/TCP、URDF/geometry、SDK、profile、限值来源、IP/固件和
非模拟反馈；记录范围只是该点的两个档位实测，不是机械物理最大速度认证。后续可传
`--speed-evidence` 复用，任何绑定变化即拒绝。`--reuse-results` 只接受相同上下文完成批次的
真实双段成功，保留原时间戳/来源；不自动续跑故障批次，不输入旧轨迹当运动命令。

## 验收证据与输出

每段必须同时满足 business-node COMPLETED、executed、final_tcp_valid、接收时间新鲜、
mode=5、停稳速度、固定朝向/目标误差和持续参考；另核对同步 joint/state/TCP 的同一时间戳。
去程和 Home 返程分别记录；任一失败不算目标成功。执行故障后只观察新鲜停稳 dwell，不强发 Home。
Stop 未确认则明确报告，保留节点 FAULT；默认不Enable，任何分支均不ClearError。
若 action goal handle 响应丢失，接受与否存在歧义，立即请求所拥有业务节点的受控 SIGINT
关闭/Stop 路径，不重发目标、不等待轨迹自然走完；客户端没有新鲜停稳 dwell 时明确记作
stop unconfirmed，不能因为节点退出就声称机械臂停稳。

每次输出目录必须不存在。`results.json` 为原子更新的总账，`actions.json` 保存完整动作结果，
`targets.csv` 列出坐标、规划、去/返结果、误差、耗时和失败原因。节点 `audit/feedback.bin` 保留
完整扭转/反馈原始记录；每次 action 的规划轨迹只作审计，永不重放。`range.png`/`range.html`
分别标出规划通过、实测往返成功、复用历史成功、规划拒绝、执行失败和待测；`REPORT.md` 给
每方向剩余边界区间。可单独运行 `arm_roundtrip_report.py RUN_DIRECTORY` 离线重绘。

## 显式断电恢复与继续

`--power-cycle-evidence /path/evidence.json` 必须绑定原fault results路径及SHA，保留本次真实
human_statement，并要求 `only_power_cycle_no_manual_joint_or_cable_motion=true`；不能编造确认。
仅execute时设置 `reference.power_cycle_handoff=true` 和 `controller.enable_after_history_restore=true`。
节点仍验证原完整有序audit、尾部故障条件、原joint join/步长/扭转容差，允许明确新controller
时钟epoch但不改历史时间。RestoreReference成功后若当前mode4，由同一业务节点官方GetErrorID
空报警检查后EnableRobot（独立ACK预算默认5s），不调用ClearError。工具等待reference_valid及
fresh mode5原 .002 rad/s/.4s静止dwell，存 `enabled_stationary_ready`，该观察不是Stop命令证明。
随后仅显式 `--recover-home` 执行low HOME，失败即结束，不重试、不强返。

`--continue-results /path/old/results.json --continuation-statement /path/statement.json` 只在本次
明确用户继续指令下接受部分/故障批次。比较Home/TCP/URDF/geometry/SDK/profile/limits/IP/固件/
策略SHA等安全不变量，源码/二进制与恢复参数差异留档。旧双段成功保留旧记录及上下文，不重新
发运动；新epoch两档资格须重做，旧资格不能覆盖新失败。原 `--reuse-results` 不自动resume故障批次。
每点preflight立即持久化；发送前等待反馈失败与真正发出后失败分开归档，任何执行故障停止整批。
