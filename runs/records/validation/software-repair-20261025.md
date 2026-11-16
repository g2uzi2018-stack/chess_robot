# chess_robot 软件修复与离线验收

日期：2026-10-25，Asia/Taipei。范围：`/home/kuang/workspace/chess_robot`。

**软件修复与离线验收完成；实机运动保持暂停。** 本轮未连接真实控制器、未运行厂家 Demo、未发送实机运动或清错/使能命令，未重放失败轨迹。历史记录仅用于解析验证。本轮未宣称任何新增实机到位或正式认证，也未提交/推送工作区中此前已有的未提交修改。

## 版本与官方复用

用户提供当前固件：`4.6.2.4-stable-d9efa2ce0-202509261512`。这是本轮版本证据，未将下载的协议版本当作固件。按照 [官方 C++ V4 适用条件](https://github.com/Dobot-Arm/TCP-IP-CR-CPP-V4/blob/0ca0b1bbd3894b951976c5dbf1535a36c173aab7/README.md)，采用 V4；没有切到 V3、升级固件或运行厂家 Demo。V4.6.6 文档中新增能力未据此自动视为 4.6.2 可用。

固定 C++ SDK：`0ca0b1bbd3894b951976c5dbf1535a36c173aab7`。在 `src/chess_robot_arm/third_party/dobot_v4/` 保存官方所需子集、MIT LICENSE、8 个原始文件 SHA256、完整 `LOCAL.patch` 及原因说明。8 个原始哈希均与 AGX 上该固定提交核验一致。

实际复用官方 `CDashboard::PositiveKin/InverseKin/ServoJ/Stop` 的命令生成、`CJointPoint/CDescartesPoint` 和未修改的 `CFeedbackData`。当前节点不再分别维护这四种命令的整段组包。继承的官方 socket 不连接；实际传输经过现有有界适配层。没有简单地把同步 SDK 全包替换进去。

官方结构按 Linux LP64 使用；编译期断言 1440 字节及 RobotMode/TimeStamp/TestValue/QActual/QDActual 偏移 24/32/48/432/480。实时反馈、历史参考审核和记录回放共用官方结构解码，只消费已经对照过的公共字段，不推断固件相关尾字段。

## 最小补丁及原因

| 部分 | 补丁 / 保留原因 |
|---|---|
| 官方编码接入 | 将两个 `SendRecvMsg` 重载设为 virtual，以复用官方命令编码而接入有界传输；上游仍同步 `WaitReply(5000)`，不能直接承担固定节拍流。 |
| 官方 InverseKin | 可选参数前补缺少的逗号。 |
| 官方数字格式 | FK/IK/ServoJ 使用固定小数精度，保留原接口的小角度精度；现场既有证据表明指数格式会被拒绝。 |
| 官方 PositiveKin | user/tool 采用原项目已验证的显式命名形式，维持 base/user=0、flange/tool=0 的契约。 |
| 拆包与超时 | 保留分号完整帧、片段/粘包缓存、16 KiB 长度上限和整次 I/O 预算；上游单次约 1023 字节 recv 不能保证完整回包。保留 TCP_NODELAY/逐次临时 QUICKACK。 |
| ServoJ 收发 | 保留异步 FIFO、精确回显与递增队列 ID，发送不等待每个 ACK。默认未确认窗口改为 6，30 ms 节拍可容纳 125 ms 回包，ACK 期限仍为 150 ms；超时、满窗口、错误回显/ID 均失败，不补发。 |
| Stop 会话 | 修正“异常后丢旧连接再抢新连接”：歧义会话锁定后保留原连接，排空已知 FIFO 有额外有界预算；损坏/缺失时仅在原连接发送一次 Stop，在停止预算内辨认它自己的回包。旧回包只留证据，不算 Stop 成功；不自动重连或重放。 |

物理连接已断、回包无法完整识别或 Stop 被拒绝时，软件不能保证远端已停止；明确返回 STOP_FAILED 并锁定 FAULT。即使收到 Stop 成功回包，执行器仍要求新鲜 mode=5、低速反馈持续 dwell 才确认停止。回包异常后会话继续禁止运动；需要显式排查/重启，而非自动恢复轨迹。

## 对外兼容与时间戳

与本轮开始前的快照比较，5 个 `.msg/.srv/.action` 定义逐字节不变（包括原工作区已有的 RestoreReference）。`/arm/joint_states`、`/arm/state`、`/arm/move` 及既有 services 的名称/类型/字段不变；单位、`base_link` 坐标系、关节顺序和 publisher QoS depth=10 不变。夹具仍只能使用 `/simulation/arm/*` 并标记 simulated，不发布到真实端点。

完整反馈帧接收后、磁盘审计和发布前，调用节点 ROS clock，保存 `Feedback::ros_received_ns`；steady clock 独立承担新鲜度/超时判断。joint_states、state.header、state.tcp、动作反馈 TCP 及实测终态 TCP 使用这一时间。TCP 从同一个 Snapshot 的关节计算。重复帧不刷新接收时间；控制器时间仅用于排序和去重，未当作 ROS header。规划轨迹的生成时间仍是规划时间，不混充测量时间。

## 速度配置与保护复核

采用 [固定官方 ROS2 Nova5 joint_limits.yaml](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4/blob/ec201c40d5db63185a3593ec9c39f80fe25aa527/nova5_moveit/config/joint_limits.yaml) 的 2.36 rad/s、4.72 rad/s² 作为离线规划基线，低档沿用其 0.3 缩放。移除 commissioning/exploration 强加的 0.02 rad/s、0.05 rad/s² 调试硬上限及 `1.5×0.02+0.002=0.032 rad/s` 瞬时停机判据。

| 模式 | 速度 / rad/s | 约 °/s | 加速度 / rad/s² |
|---|---:|---:|---:|
| low（默认） | 0.708 | 40.56 | 1.416 |
| high | 2.36 | 135.22 | 4.72 |

`profiles.default`、`profiles.low/high.speed_rad_s`、`acceleration_rad_s2` 均可配置；空 Goal.profile 选择默认档。现有 conservative、commissioning_slow 和探索阶段名继续接受，相应模板使用 low 时间参数。逐关节 `limits` 表示厂家规划上限，规划取模式请求与逐轴上限的较小值；不再叠加一套低速调试封顶。

3 rad/s ≈171.89°/s，超过此固定官方 Nova5 规划上限 2.36；本轮不强行设置 3。**2.36 是所采用官方配置的上限，不是已经读出的本机合法物理极限。** 产品代际/控制盒、当前控制器再现/缩减参数尚未核实，不能编造本机合法上限或支持 3 rad/s 的结论；这属于恢复运动前的待核项。更严格的本机范围优先。

ServoJ 没有在本调用中使用 rad/s 速度参数；速度由轨迹时间律产生。未另发 SpeedFactor/VelJ/AccJ，不能把 SDK 30% 等同于 0.3 rad/s，也不能声称这些比例必然限制 ServoJ。当前两档明确使用 rad/s、rad/s²；官方 SDK 中角位置在边界换算为度。

保护复核：碰撞包络、桌面、关节余量、奇异性、净/总扭转、参考连续性、最终到位误差和停止 dwell 均保留。geometry_guard.cpp、反馈帧验证和原有 net.hpp 与本轮开始前逐字节一致。高/低速度别名在探索模式仅使用 near 空间边界，不扩大空间授权。跟踪保护统一为已有正常配置 0.15 rad（原极低速模板为 0.005）；停止/到位的 stationary 门限和 dwell 未提高。探索 ServoJ 跟随时间默认回到发送周期 30 ms，未保留旧 200 ms 低速调试组合。

高档 2.36×0.03≈0.0708 rad/周期，仍小于 0.1 rad 的关节参考步长保护；0.1/2.36≈42.4 ms，意味着高速连续丢帧仍可能因相位/步长不可信而停止，未通过放大参考跳变限值来掩盖。跟踪保护、包络和离散采样不构成高速实机认证。

## 验收证据

AGX / Ubuntu 22.04 / ROS2 Humble 编译通过。Arm **55/55**，Twist **20/20** 通过；最终记录 `runs/software_repair_20261025/build05.log`、`unit05.log`。编译仍有此前已有的 Plan selection/optional 静态警告，未计为测试失败。

ROS 回环最终 `loopback03/result.json`：PASS，17 类事件，包含低/高/空 profile、完整抓取门控、取消/完成竞争、负确认/超时不撤离、125 ms ACK 不阻塞发送、200 ms ACK 判失败后同连接 Stop、错误回显、Stop 拒绝锁定。新增“成功 ACK 但冻结编码器”必须失败；重复反馈跨多个发布周期保持原接收时间，joint/state/TCP 一致。55 项单元回归另含 12 KiB 分片回包和 17 KiB 越界拒绝、速度导数、暂停门禁及参考历史测试。

| 仅离线反馈文件 | 总帧 | 唯一帧 | 重复帧 | 所有使用字段对照 |
|---|---:|---:|---:|---|
| exploration physical07 | 28,627 | 28,624 | 3 | 全部一致 |
| neutrality physical01 | 21,780 | 21,777 | 3 | 全部一致 |
| 合计 | 50,407 | 50,401 | 6 | PASS |

回放工具 `test/replay_feedback.cpp` 不实例化 Driver/socket，只读文件。physical07 的旧峰值仍为 0.0369114127085 rad/s（沿用已有 °/s→rad/s 解释），没有过滤/裁剪；字段数值一致不等于已验证瞬时速度滤波/采样语义。记录 SHA256、原始 SDK 哈希核验、接口哈希及回放明细见 `verification.json` 与两个 `replay-*.json`。回放不恢复参考、不发送运动，也不新增到位结论。

## 恢复实机前仍需验证

1. 现场核对 Nova5 具体代际/控制盒及当前 4.6.2.4 的再现、缩减、速度/加速度合法范围，确认当前反馈速度字段语义，按更严格的真实范围配置两档；未确认前不宣称 3 rad/s 合法。
2. 保留真实绕线历史，显式确认记录空窗与停止端点；本轮未初始化、恢复或清零任何真实参考。TCP/负载/几何和认证状态仍待原要求验证。
3. 另行授权后用新目标、分档验证真实跟踪、调度尾延迟、停止距离/时间、同连接 Stop 异常链和实际到位；禁止重放旧失败轨迹。不得把本轮回环或规划通过当成实机证据。

保存的 `motion.paused=true`、`hardware.enable=false`，正式认证布尔值均未改 true。本轮只完成所要求的软件和离线阶段。
