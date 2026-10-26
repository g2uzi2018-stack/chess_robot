# 反馈空窗独立诊断（2026-10-26，0d5a2c9基线）

结论：旧146.428485 ms单次空窗尚不能归因于SDK bug、控制器或链路。用户明确允许把本次反馈新鲜度窗口改为200 ms；这改变故障判据，不证明机械臂、网络或软件已正常。旧fault、原始audit和原100 ms上下文均保留。

## 已确认事实

- 原 `physical_continue01/audit/feedback.bin` SHA256 `9df270e7e11e9de866848d2a9e6288bdba43197255935a86ed524028ac41acb4`，1721完整1440字节帧，主机记录中位间隔8.173824 ms，最大146.428485 ms；mode4/5/7分别606/671/444，无mode9、无控制器stamp重复或回退。
- 最大空窗两侧controller原stamp差46；下一帧主机仅137.595 µs，controller stamp差100。该字段不作为ROS header时间，也没有同步换算证明它等于帧生产时刻。
- 当时全部74个资格ServoJ已ACK，空窗在末条ACK后0.732127 s、arrival/dwell阶段。原100 ms新鲜度保护中止资格，Stop58.781728 ms、后续fresh静止dwell0.402554 s确认。动作没有成功；当前epoch两档资格必须重新实测。
- 原始UTC audit只能看到完整帧消费附近时间，不能区分内核包到达、应用调度、同步审计写入或控制器生成。以新200 ms离线重算空窗超限计数为0，只表示该记录不超过新判据，不抹除146 ms空窗。

## 源码对照

官方pin `0ca0b1bbd3894b951976c5dbf1535a36c173aab7` 的 `CFeedbackData` 与字段偏移继续复用；live数据流**不运行**官方 `CDobotClient::WaitReply`，该继承网络未连接。因此仅以官方同步SDK的select/recv实现来解释本次反馈空窗不成立。

实际路径是 `Socket::exact(1440)` → 完整帧steady/ROS采时 → 同步 `feedback_audit_.write` → 校验/sequence/官方结构解码 → `State::update`。关键候选：

- `exact`正确循环到1440字节，单次recv不会跨读下一帧；未发现固定100/146 ms拆包sleep。每次receive后重复TCP_QUICKACK setsockopt，开销需测，不能臆测其引起或解决空窗。
- audit的两次ostream写入在状态交付之前；缓冲刷新可能同步阻塞。旧记录没有写入耗时，不能排除或归因。
- `State::update`与snapshot共用mutex，锁竞争、模型/扭转计算或线程调度也可能延迟消费，需按层采时。
- `node.feedbackLoop`在snapshot不fresh时固定10×10 ms退避，可能放大首次空窗之后的恢复延迟。它不足以解释首次空窗来源，也不应仅为诊断删除。
- 原feedback.bin使用system_clock，而保护使用steady_clock；旧资料没有逐帧成对mono/UTC时间，需排除主机UTC校时影响。不可改写旧stamp来“修复”结果。

## 最小低干扰证据

主代理拥有C++修改；诊断默认关闭，通过 `diagnostics.feedback_trace` / 工具 `--feedback-trace` 显式启用。建议固定内存环/预留容量、单线程写结构、退出统一CSV，记录容量/丢弃数，不逐帧printf、不每帧flush/fsync；保留原审计数据与原ROS采时语义。

|层|必要记录|能回答的问题与局限|
|---|---|---|
|socket exact|frame index、read begin/end steady、recv次数/字节与poll/recv/QUICKACK累计耗时|长时间花在等字节或应用内部；等待时间仍含未获调度|
|audit/解析|audit begin/end、decode end、同点UTC+steady|排除同步IO/解析耗时或UTC跳；不等于内核收包时刻|
|状态交付|update begin/end、loop backoff begin/end、线程CPU累计值（可选）|锁/计算/显式sleep与被抢占线索；单靠wall/CPU差不能区分阻塞位置|
|主机内核收包|同一次连接的pcap/TCP seq、重传、包到达时间、抓包drop计数|内核已连续收到而应用迟读支持主机消费层；内核同样空窗仍不能区分上游控制器/线路|
|控制器/链路|控制器侧发送日志或交换机镜像，双端TCP seq关联|区分发送本身间隙与传输丢包/延迟；单侧抓包不能证明控制器生产连续|

需要时仅在原业务连接上被动抓包，不再打开第二条反馈socket。固定小snaplen/反馈端口过滤、输出到独立目录并记录capture drops；无抓包权限则明确证据不足。抓包时间通常也是主机软件时钟，不能假定是网卡硬件生产时间。调度perf/ftrace等较高开销诊断只在低开销trace仍无法解释后另行评估，不在实测期间编译或高CPU重绘。

判断规则：read长且内核包连续→进一步看调度/锁/IO；audit长且read本身短→审计阻塞证据；完整帧read短但decode/update长→应用层证据；内核与应用同样空窗且有重传→链路线索；无重传、controller stamp也跳→仍不足以认定控制器生产问题。任何新数据均需与源码SHA、真实/模拟、门限和命令时间线绑定。

## 已提供工具和验证

`tools/analysis/feedback_gap_diagnostic.py` 纯stdlib离线解析，不导入ROS、不连socket、不发送命令。它检查每条原始1448字节audit记录与V4帧头，输出各文件独立空窗，不隐式拼接时钟epoch；可读可选timing CSV、保留列和SHA，仅对显式duration列统计，不跨时钟相减。

```bash
python3 tools/analysis/feedback_gap_diagnostic.py \
  --audit runs/power_recovery_20261026/physical_continue01/audit/feedback.bin \
  --threshold-s .1 --out runs/new_offline_feedback_analysis
```

实测原记录复算：`runs/feedback_gap_20261026/offline100ms/`有1次超限，`offline200ms/`有0次，最大间隙和原文件SHA相同；不据此宣称物理正常。工具窗口/继续门禁27项单元通过，记录 `runs/feedback_gap_20261026/tool-unit01.log`。

本轮Python补丁新增 `--feedback-freshness-s`（默认.1）、`--feedback-window-evidence`（仅真实原话与原fault SHA精确授权.1→.2）、`--power-cycle-join-audit`（必须与power证据SHA一致且在完整13链）、`--feedback-trace`。仅窗口这一处安全差异可授权，其余不变量严格一致；授权后报告 `safety_invariants_equal=false` 和具体delta。旧14成功保留旧上下文，旧资格不能当新epoch资格。最新无外部运动声明与旧断电声明分别归档。实机操作由主代理唯一控制，本独立诊断没有启动任何控制器连接或运动。

## 真实只读窗口与启动检查

`handoff_readonly200_01` 已退出，完整13份历史恢复PASS，保留非零net约-0.516379/total0.520377 rad；live mode5，HOME只有plan_only，无Enable或运动。4 s只读窗口491帧，raw=trace、drop0，最大host间隙8.842852 ms；audit max1.571877 ms、dispatch max0.133054 ms。没有pcap，不外推运动期间或偶发故障。恢复后及最终state/TCP/joint采样stamp一致；启动前第一次被动读取的旧joint消息不当作同快照证明。派生分析在 `runs/feedback_gap_20261026/readonly200-analysis01/`。

首次200 ms入口 `physical200_01` 因节点遗留exploration≤.1内部参数cap而在startup被拒绝，无SDK连接/audit/action/运动，不计执行故障或实际测试；主代理定向修为仅exploration≤.2、commissioning仍≤.1，并测试.2接受/.200001拒绝。后续采用新 `physical200_02` 目录，原01保留。

## 官方primary文档与固定pin接收示例

固定pin的[官方README](https://github.com/Dobot-Arm/TCP-IP-CR-CPP-V4/blob/0ca0b1bbd3894b951976c5dbf1535a36c173aab7/README.md)适用V4及以上控制器，描述30004标称8 ms反馈、30005为200 ms、30006默认50 ms；当前[官方ROS2 V4 README](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4)也配置29999控制、30004反馈。本项目实际端口是30004，并未连到200 ms的30005。查阅的官方资料没有主机到达间隙的最大值/抖动容许或硬实时保证；8 ms不能自动转成到达deadline。没有厂家依据把146 ms认作“机械臂本来如此”，也不能单凭超出8 ms就认定SDK缺陷。

固定pin的[官方示例](https://github.com/Dobot-Arm/TCP-IP-CR-CPP-V4/blob/0ca0b1bbd3894b951976c5dbf1535a36c173aab7/Cpp/DobotTcpDemo.cpp)连接反馈30004；示例上层每10 ms复制最新数据，该sleep不是控制器发帧周期或最大间隙承诺。[CFeedback::OnRecvData](https://github.com/Dobot-Arm/TCP-IP-CR-CPP-V4/blob/0ca0b1bbd3894b951976c5dbf1535a36c173aab7/Cpp/api/Feedback.cpp)使用4320字节缓存、Receive追加、头/magic扫描、每循环解析一个1440帧再移动剩余字节。它与官方Dashboard的WaitReply不同；项目没有运行这条厂商CFeedback接收循环，只复用CFeedbackData字段。项目exact/同步audit/State路径与上游缓存逻辑应分别审计；未运行的上游接收实现即使存在潜在问题，也不是本次故障的已证原因。厂家Demo仅阅读，未编译或运行。

## 已取得的新模拟时序证据

主代理build后授权的域98 localhost power回环 `runs/feedback_gap_20261026/power-trace-loopback01/result.json` PASS；默认拒绝旧fault尾且Enable=0，显式分支经业务节点模拟使能一次、保留非零0.0024 rad历史，再low Home和两档资格成功。全部simulation，真实运动0。742条原始帧与742条trace一致，capacity262144、dropped0、退出join后flush；dispatch/audit markers有效，无frame序号回退。原同快照stamp机制未修改。

`power-trace-analysis02/analysis.json` 中，模拟最大complete gap44.403793 ms发生在frame244→245。前帧audit阶段wall耗时44.205461 ms，占该间隙绝大部分；前complete到下一read begin44.247732 ms，而下一read仅0.156061 ms。这是**同步audit阶段或该段被抢占可延迟消费**的直接观测；尚不能把wall时间全部归因于磁盘IO，也不能证明旧146 ms由同一原因产生。

本模拟全批audit median75.823 µs、p99 240.188 µs、max44.205461 ms；dispatch max427.897 µs，decode max2.88 µs，read max31.040413 ms。UTC/steady相邻gap之差max9.472 µs，未见明显UTC阶跃。诊断分析会同时输出前帧audit/dispatch/backoff与后帧read，避免把后帧完成之后的audit错误归入之前空窗。

时序CSV分析支持固定内存trace及 `feedback_timing_meta.txt` 的rows/dropped校验；`--pcap FILE --feedback-source IPv4 --feedback-port PORT` 使用标准 `tcpdump -tt -nn -S -r` 仅离线读取入向TCP payload，将长complete gap与同UTC窗口的payload包数/字节和邻近包间隙并列。它不重组TCP、不把包数当帧数；多连接、截断、抓包drop和主机软件时间戳都必须保留为限制。

## 闭合实机actual02的独立结论

`physical200_02` 已正常退出、总账COMPLETE；有限清单38点为14历史复用、9本批新往返成功、15规划拒绝，low/high资格各两腿通过，12个边界槽用完。这里仅用于标识诊断窗口，范围验收细节由主报告保存，没有为诊断追加运动。

独立离线分析 `runs/feedback_gap_20261026/physical200_02-diagnostic02/` 对照原始audit、退出flush的trace及同一真实反馈连接pcap：

|观测|结果|
|---|---|
|记录窗口|trace steady约154.068 s；raw与trace均18,783帧|
|接收间隙|raw最大42.952764 ms，median8.177140 ms；accepted帧最大steady间隙42.953180 ms|
|100/200 ms超限数|原始帧与accepted帧均0/0，本次成功没有依赖容忍一次100–200 ms间隙|
|反馈状态|mode5=4,590、mode7=14,193；无mode9、无stamp回退；4重复stamp按原规则忽略，不刷新有效反馈|
|trace完整性|accepted18,779+duplicate4，capacity262144、dropped0、rows与raw相符；无backoff记录|
|audit wall|median73.006 µs、p99 174.973 µs、max31.481984 ms，总约1.722 s（窗口1.12%）|
|其余阶段wall|read max42.823838 ms；decode max0.380607 ms；dispatch max0.700403 ms|
|抓包|eno1，37,566 captured/received-by-filter，kernel dropped0；单一30004→AGX连接，18,791入向payload均1440 bytes|
|TCP payload范围检查|按tcpdump绝对sequence，未见payload区间重叠或向前缺口；不作完整TCP重组/链路健康认证|
|内核软件抓包时间间隙|payload最大42.966 ms，中位8.173 ms|

两类不同证据必须分别解释：

1. **实际最大间隙在读取/到达侧。** frame6267→6268的steady complete gap42.953180 ms，前帧audit仅0.077375 ms，前complete→nextread begin0.129342 ms，下一read42.823838 ms；同一UTC窗口内核包间隙也约42.966 ms。故这次最大间隙不是同步audit或解析独自制造。单侧软件抓包仍不能区分控制器产生/发送、线路与网卡/主机网络栈或内核调度延迟；没有控制器侧发帧证据，不说“机械臂本来如此”。相邻controller原stamp增量有21/29/24/35等非固定8的情况，只作原始辅助数据，不能当同步生产时钟。
2. **同步audit临界路径也确实造成应用延迟窗口。** frame1880→1881的complete gap31.629821 ms中，前帧audit wall31.481984 ms、下一read仅0.095198 ms；同窗内核payload连续约8.199 ms，3个payload已到。这将该窗口定位到应用audit阶段或该段被抢占，不能把31 ms全部称为磁盘IO耗时。其它前audit约22–27 ms窗口也有相同形状。模拟44 ms结果与它一致，但两者都不能倒推旧146 ms的唯一原因。

因此，当前可判定的是“存在应用审计阶段wall延迟，同时存在单侧抓包也可见的到达间隙”，不是某一个统一SDK缺陷。本次未复现100 ms以上间隙，不能因此证明偶发问题已消失；旧146 ms无pcap/阶段trace，根因仍未定位。抓包drop0及sequence范围连续仅限本次捕获窗口，不能排除未观测路径/时段问题或把控制器标称8 ms升级为硬实时保证。

**修复判断：** 没有证据要求修改官方命令/数据结构或未运行的CFeedback接收器；不为猜测改变TCP设置、控制器配置或继续放宽门限。本次新增默认关闭的内存trace已提供必要定位。同步audit在关键路径上出现几十ms wall延迟有实证，值得后续单独优化：若移动到有界审计写线程，必须保留原接收时刻与顺序、无静默丢帧、队列溢出/写失败明确使任务失败并按原保护停止，并做负载/停机归档回归；仅增加缓冲或移除保护不足以证明解决。鉴于原146 ms未定位及本批已完成，不把此优化冒充已修复旧根因，本轮不再改动运动源码。

原raw SHA `9b72e5449ab1803590efcc06dfaa41681480492bf945a9ebfb88b4da5736e92a`；trace SHA `4f9cd39caf6b07ce24fc902698bf98db5455d025ed148bc164f0d56e926c4178`；pcap SHA `c75c4b4d3d981483a68430cc5b8bb4a05623384cca2d5b811751cc229dcae31d`。派生目录保留analysis.json、gaps.csv、timing-frames.csv、timing-gaps.csv和capture-payload-packets.csv；capture原日志也绑定SHA。旧fault结果、记录与门限上下文未覆盖。
