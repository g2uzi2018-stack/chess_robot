# Home → 实际 TCP → Home 范围验收：2026-10-26

**BLOCKED / EXECUTION_FAULT_STOPPED。未完成范围探索；保持实机暂停。**

本轮在用户明确授权“先按照1rad每秒走”、断电后“先回到home然后继续剩余未检测的[点]”，并确认“只有断电，未手动移动或改线”后，仅启动了一次继续批次。所有运动均经真实业务 `/arm/move`；没有独立运动维护程序、ClearError、自动清零或故障返 Home。普通/低档请求分别为 1/0.3 rad/s、1/0.3 rad/s²；这是本次用户策略与规划上限，**不是本机硬件限值认证，也不代表各段实测速率达到 1 rad/s**。

## 结果与剩余项

|阶段|结果|
|---|---|
|2026-10-25 历史|14 个不同目标实测去返成功，保留原腿记录、时间戳和来源；今日不重扫、不计为新增到位|
|断电历史衔接|原12份 audit（末份包含原mode9尾2帧）恢复，非零扭转保留；当前控制器时钟作为显式新epoch，不改写历史时间|
|恢复正式 Home|节点官方 GetErrorID 空报警后 EnableRobot 一次；等待 fresh mode5、原 .002 rad/s/.4 s 静止 dwell、reference_valid 后，low HOME 实测通过|
|今日剩余粗扫预检|12 点：1 点规划通过（upper_front，未实测），11 点规划拒绝（含 upper_back）；逐点立即保存，不把规划当到位|
|新epoch低档资格|left_150 去程执行反馈故障；受控 Stop 与新鲜停稳确认通过，未执行返程|
|新epoch高档资格、普通目标与边界|均未开始；今日新增成功目标0，边界槽0/12|

固定粗扫26点，六轴方向偏移 .15/.30/.60 m，加上下 .15 m 高度切面各四点；最多12个边界槽，总38个不同目标，初始估时10–25分钟。今日预检通过目标的预计去返纯运动总和9.72 s（不含资格、SDK、dwell），故障后立即结束，无无限扩点。

以下只表示**历史最远已测偏移**及粗扫规划拒绝区间，不是最终可执行边界，更不是全局工作区证明：

|基座方向|历史最远实测偏移(m)|最近较远粗扫拒绝偏移(m)|本轮边界槽|
|---|---:|---:|---|
|left/right/front|0.30|0.60|各0/2，未完成|
|back/up/down|0.15|0.30|各0/2，未完成|

upper_front、upper_back 至今均无去返实测成功。后续仍须先解决反馈连续性故障、重新建立合法参考与新epoch两档资格，才能考虑继续；本次不自动恢复或重放。

## 实测精度、时间及停止

正式 Home 沿用项目两份相同 YAML，SHA256 `165f3ec9727f78bd5069538aa0029fd96275574dfd882db472a08b36a1be634a`，TCP `[0.0303514019,-0.5765188179,0.2749875911]` m；无预抓偏移或额外动作。

今日 low Home：1.281975 s、TCP误差0.030145 mm、朝向误差0.00005684 rad，独立恢复阶段不算目标成功。历史14点的去程TCP误差0.3115–0.3961 mm、返程0.00408–0.03971 mm；去程耗时2.404–8.210 s、返程2.395–8.140 s，完整逐点值见CSV与原记录。

今日资格 action 返回 code13 `feedback lost during task`。全部74个预期 ServoJ 已ACK，终点指令与规划末关节一致（1e-9 rad内）；最大 Servo ACK20.179 ms、最大待ACK数1。故障发生在末条ACK后0.732127 s的到位/dwell阶段，不能算资格到位。Stop回包 `0,{},Stop();`，58.781728 ms；fresh mode5原停稳观察持续0.402554 s后 confirmed=true。最终TCP约 `[-0.1196422,-0.5765217,0.2746432]` m，**未在Home，reference_valid=false**；失效时零扭转字段是无效状态占位，未执行清零。

## 离线故障结论

原始1721帧中 mode4=606、mode5=671、mode7=444，无mode9。最大主机接收间隙146.428485 ms，超过原0.1 s新鲜度门限：frame1604→1605，host ns `1792976912061889997`→`1792976912208318482`；同对控制器时间增量46 ms。下一帧主机间隔137.595 µs而控制器时间增量100 ms。重复和回退stamp均0；两侧mode5，最大关节差3.3623e-6 rad。形状与批量迟到/追赶相符，但**网络、控制器、主机调度层根因尚未定位**；没有删除空窗或放宽门限的依据。

未见本次command传输关闭/回包超时证据；它不是goal-handle等待超时。业务节点的反馈新鲜度保护触发，取消当前动作并Stop，符合保守失败处理。不能因StopACK或后续反馈恢复而声称空窗期间已持续安全。原10/25批次的mode9原因也仍未定位，原 `EXECUTION_FAULT_STOP_UNCONFIRMED` 保留，不改写为今日状态。

停止后父代理只读 GetErrorID 为 `0,{[]},GetErrorID();`（2.17212 ms），无新使能/运动/清错；这不证明反馈连续或可以重试。eno1累计RX dropped118559没有故障前基线；故障窗口内核journal无entries，均不足归因，未改网络/调度配置。

## 官方复用、最小改动和验证

沿用固定官方V4 C++ SDK `0ca0b1bbd3894b951976c5dbf1535a36c173aab7` 的命令编码、CFeedbackData、FK/IK/ServoJ/Stop，并复用官方 GetErrorID/EnableRobot。固件原话为 `4.6.2.4-stable-d9efa2ce0-202509261512`。SDK目录相对修复基线bd221ccc无新增差异；厂家Demo未运行。

最小断电补丁默认关闭：原始mode9只允许连续尾段，故障前原新鲜度内需静止正式Home；live4/5接入仍执行原joint join≤.001 rad、步长/界限/扭转/home容差。明确power evidence绑定原fault SHA；RestoreReference成功后显式使能分支才用官方空报警检查与EnableRobot，独立ACK预算5 s；无ClearError。工具显式continuation比较安全不变量，允许源码/二进制恢复修复但记录差异，旧14记录保持旧上下文，不把旧资格当新epoch资格。去返预检、反馈同快照stamp、碰撞/奇异性/关节/扭转/跟踪/停止保护均保留，接口五文件与Home未变。

父定向审计 `runs/power_recovery_20261026/parent-verification.json`：60 Arm+20 Twist C++、25工具单元、power回环通过；Enable=1、Stop=1，无ClearError/MovJ/MovL。回环验证默认不使能、显式mode4恢复→节点GetErrorID/Enable→fresh mode5原dwell→low Home及新资格，非零历史保留。模拟证据单独存档，不作为真实通过。

实际工具SHA `96f7f01ff7e3670b0cd60576816e0e9238f3286b92a5773aa5a6a8e9b9d7746c`，实际node二进制SHA `2988a863215995834497bd6aaae88a35acf897b0f3d8852af82c249b4d2e6c74`。运行后仅修补资格失败的try/finally归档：交付工具SHA `d9b7dc281aee17ae86a0ce89b84d2f0d98ffb69164601584fcccf26d1155cb1f`，25单元通过；此补丁未重新实机验证。原总账中失败资格仍位于targets、qualification_trials为空；合并视图单独提取该事件，不篡改原文件。

## 交付与原始证据

- 本目录 `combined-range.png` / `.html`：历史14绿色成功及今日同坐标红叉同时显示；upper_front蓝色规划通过未实测、其余11灰色规划拒绝；边界未完成。
- `combined-evidence.json`、`combined-targets.csv`、`qualification-events.csv`、`direction-boundaries.csv`：26点总账及独立失败资格事件、原始来源SHA。合并视图是派生证据，不是可执行续跑文件。
- `runs/power_recovery_20261026/physical_continue01/`：原results/actions、trajectory、audit、reference_restore、enabled_stationary_ready、Stop证据；启动命令与源hash在上级 `launch-continue01-command.json`。
- `runs/power_recovery_20261026/physical_continue01_offline/`：analysis.json、gap-detail.json、gap-surrounding-64.csv、feedback-last100.csv。
- `runs/power_recovery_20261026/{power-cycle-evidence.json,continuation-statement.json,continuation-preparation.json,actual-continuation-safety-check.json,parent-verification.json,GetErrorID-after-fault.txt,eno1-after-fault.txt,kernel-fault-window.txt}`。
- `runs/home_roundtrip_20261026_failure_analysis/`：原10/25故障分类及原始记录稳定性；旧10/25报告保留为历史。

原results SHA：10/25 `6114b227dbdbc858f82050ef37ba349948dfb6750e60d97e5dcc163ae34922a1`；10/26 `0f64e03180d3b89b26a830c9a84a057427da2a02caaa4f8a7da310fc80a4a1e9`。当前没有活动arm_node/fixture/测试进程。未完成项是反馈空窗根因、新epoch资格、剩余实际点位和六方向边界加密，不宣称整体完成。
