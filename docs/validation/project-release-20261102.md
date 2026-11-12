# 项目整合、Home解耦与发布验证 — 2026-11-02

## 1 发布身份与结论

首次整包部署源码为 `4d3a96cfa5c5e834b5bbf96282077ea740c3e1f8`，该阶段主分支和夹爪分支 `feat/gripper-node-20261025`（`16af8bd`）已原子推送。夹爪分支历史保留并合入主分支；源码文档阶段为 `f5a2718`，Home解耦代码阶段为 `4ce936b`。
整包发布目录为 `/home/kuang/workspace/chess_robot/runs/project_release_20261102_01`（下文简称RUN）。随后面板配对修复提交为 `2645986040cc4f6817200cc44b061078f48c84a9`，专项目录为 `runs/panel_pairing_20261102_01`（简称P），已仅切换console部署。本页以2645986后的主分支文档为基线补记；最终远端main以交付推送回执为准，后续文档提交不改变已受测二进制身份。

源码整合、12包构建、离线/mock验证及五项service切换完成；后续发现的面板配对缺陷已修复并完成独立复核和现场只读复验。机械臂新目标与旧历史基准已加载，非零扭转保留，原审计完整归档。现场检查为无动作健康检查，不代表自动夹取、实机运动范围或识别准确率验收。

## 2 源码与离线验证

| 范围 | 最终结果与计数方式 |
|---|---|
| ROS包 | 12包增量构建完成；使用既有SDK，不自动安装/替换SDK |
| 独立gtest | 204例：113 arm、31 checkpoint、20 twist、20 gripper、18 handeye、2 camera，全部通过 |
| 首次整包其他已注册测试 | 14项手眼集成、4项validation自检、2项panel链路通过；面板专项临时验证另见第5节 |
| 首次整包colcon摘要 | 230，包含CTest wrapper与嵌套gtest重复计数，不能写成230个独立单测 |
| 独立维护工具 | 12个C++用例、7个配置用例及协议loopback通过；不属于ROS colcon自动重建范围 |
| 其他回归 | 83视觉CPU用例、13夹爪gateway场景、6项homing检查、13历史场景、27 roundtrip unit，以及对应arm ROS/roundtrip/powercycle localhost集成通过 |
| Home兼容 | 原serializer写档/新实现读档及真实checkpoint不可变副本验证通过；原identity、local、previous_phase、端点保持，改历史基准拒绝 |
| 独立审查 | Home解耦最终补丁经Gemini/Cursor CLI静态复核并由主代理验收；执行证据由实现/集成方提供，不等于审查者重跑全部测试或覆盖所有运行问题 |

首次全包配置缺少realsense2_DIR；随后使用既有 `/home/kuang/.local/realsense-2.58.1` SDK的固定配置路径重建，没有安装或替换SDK。构建配置和初次日志继续保存在RUN下 `build-all.log`、`build-sdk.log`与构建缓存，克隆源码不等于已具备该外部SDK。

上述范围存在交叉复跑，不能相加得出一个独立测试总数。最终registered结果见RUN下 `test-final-results.txt`；维护工具重建见 `operator-rebuild.log`，工具SHA256为 `2b01df196758579bf7d8479207bfac4c30b51be570639ce08a70f5cd4dc62b32`，旧二进制保存在 `before-tools/`。

保留初次失败：增量隔离测试中，panel重启后的准入在8秒窗口超时，见 `test-home-results.txt`。补充失败状态输出后，定向重复和最终registered测试通过，未放宽门限，原因没有据此归咎于现场设备；原始失败未删除，定向记录在 `panel-restart-diagnosis.log`。

## 3 部署来源与服务生命周期

| Service | 本次检查MainPID | 来源 |
|---|---:|---|
| `chess_robot_arm.service` | 379459 | RUN/install的arm_node |
| `chess_robot_console.service` | 414279 | RUN/deployment/start_console.sh；panel增量切到P/install，夹爪继续使用整包安装 |
| `chess_robot_preview.service` | 381998 | RUN/deployment/start_camera.sh管理RUN/install中的camera/preview |
| `chess-robot-xiangqi-render-20261031.service` | 382804 | RUN/install中的tracking_preview |
| `chess-robot-xiangqi-preview-20261031.service` | 382810 | RUN/source中的Python监督入口及perception/scene |

五项均为active/running。ROS图中8个功能节点为arm、gripper、panel、vision、preview、tracking_preview、xiangqi_perception与xiangqi_scene，另有本次临时只读探针。12包构建不表示12个常驻节点；手眼执行工作流和validation专用工具未因本次健康检查自动启动。PID只是本次核对快照，不能用作未来kill或版本选择。除后续panel来自P/install外，其他已核对C++可执行文件来自RUN/install，Python源码入口来自RUN/source；Python训练运行环境、模型和TensorRT依赖仍为已固定的外部工件，不表示所有依赖都已打包进RUN。

旧arm/console/识别/渲染unit是transient，停止后被systemd回收，曾出现Unit not found。本次为这四项登记同名持久user unit，并在 `~/.config/systemd/user/<UNIT>.d/50-project-release.conf`指向RUN/deployment脚本；相机原持久unit只作drop-in覆盖。daemon-reload后切换成功。
持久定义与开机启用是两件事：本次未执行enable，四项登记记录为未启用；不承诺机器重启后整套自动恢复。旧配置和运行选择备份在 `RUN/deployment/backup/`，变更记录见 `deployment/config-result.json`与 `deployment/registered-units.json`。

视觉保留同一严格FP32 TensorRT引擎和全图候选＋ROI策略，未上线轻量候选模型或深度生产模式。沿用既有managed fixed-clock acquire/release钩子，未更改功耗模式；不是关闭或重写原时钟管理策略。

## 4 Home、历史与零命令切换

读取运行参数确认日常 `home.joint_positions_rad` 为新采集值，`reference.joint_positions_rad`仍为原历史基准，`feedback.publish_hz=100`，`checkpoint.enable_after_restore=false`。新启动命令还显式关闭旧的history-restore使能选项。

参考恢复有效且没有清零：切换记录total由 `0.3209050356575259` 到 `0.32090419912133694` rad；这是停稳真实反馈更新下的小差异，不要求周期存档字节永远不变。另一个不可变副本验证记录identity为 `00a5beae3ca7438dea8476d30b79426472fa7450c272a254ba6cf6b129da5449`，不改原档、无设备连接，并完整保留phase/local/端点。

旧 `commands.csv`被原轮转机制完整归档到`.1`，旧SHA匹配；新活动文件只有表头，命令记录为0。因此结论是“旧审计完整归档且新会话零命令”，不是“活动commands.csv哈希未变”。
没有发送真实使能、运动、Home、夹爪初始化或开合指令，也没有调用初始化参考来抹去历史。checkpoint由原服务周期更新，其自然变化不作为动作证据。

## 5 现场只读窗口

以下为2026-11-02约20:45（Asia/Taipei）的部署后采样，时间戳和样本数以原始JSON为准。

| 层 | 观测结果 | 本次可以说明的范围 |
|---|---|---|
| 机械臂 | ArmState/JointState约100.37Hz，fresh/reference=true，mode5，非模拟，busy=false | 新节点、反馈、参考与参数正常；不证明新Home实际回程已执行 |
| 夹爪 | 约10.05Hz，connected/feedback_valid=true，非模拟，init=0，raw1000 | 通信和反馈正常，保留未初始化状态；MOVE前提未满足，不称运动就绪 |
| 相机 | 约30.00Hz，stream_ok=true，彩深帧号对应，曝光25000μs/增益16 | RGB-D采集继续；不是完整模型识别率 |
| 相机测量/scene | 约30.07/30.09Hz，协议可解码，窗口200个测量包均0检出 | 仅为本次零检出窗口的链路吞吐；不当作四子或满棋盘模型FPS |
| 渲染 | 9次状态均解码，节点自报video_hz约30，visible_tracks=0 | 状态与视频链路存在；状态topic约0.5Hz不是图像帧率 |
| HTTP | 8088根页、8089/api/state、8089/api/arm/state均200 | 路径可达；不能由HTTP 200推导设备动作资格 |

本窗口相机几何目标资格为false，scene基座变换可用，motion_ready/grasp_ready均false。0检出不证明实物场内一定没有棋子，也没有在本次重新验收模型准确率。近期四子约17.2fps和轻量候选＋ROI约35ms仍是各自旧窗口/离线阶段证据，不能被本次约30Hz替代。

首次面板快照出现arm `matched=false`、`can_move=false`，原因是状态与关节时间戳未匹配。追加100次只读GET仅10次匹配/准入，90次被该门限阻止，确认存在实际问题，不能写成偶发单次快照。旧实现只保留两路各自最新一条并使用QoS深度1，100Hz下丢掉本可配对的帧。底层arm反馈健康或HTTP 200不等于面板关节控制可用；后续仅重启console修复，其他已验证节点未再次重启。

### 面板配对修复与最终复验

修复提交 `2645986`仅改panel node.cpp，源码SHA256为 `999a5b6a76a9eb26e0fcd35ccf9a69621ee6e6ca34fcb49e90c7b670bf1d9ac3`。QoS20、两侧各32帧有界缓存只取原stamp完全相同的真实pair，相对最新流最多落后100ms，原1秒source/receive时效保持。任何最新不安全/无效、同一路重复或回退时间戳都清空pair/cache；arm source未来时间严格拒绝。

Gemini初审发现单路safe可能复活故障前pair，主代理复现并修正；故障后必须有采集stamp严格晚于观察unsafe的ROS时刻的新完整安全pair才能恢复，排队迟到的故障前pair不能解锁，故障恢复锁也阻止夹爪互斥入口。最终Cursor/Gemini复审DONE、无剩余发现，collect无scope/git drift，主代理验收；审查是静态检查，仍假设两发布者与panel使用同一个ROS/系统时钟域。

本次原2项ROS/HTTP集成和1项临时配对回归均通过；临时检查覆盖16类条件，100次mock GET全部ready、零Action、零控制器连接。临时fixture与新增第3项CMake注册已经删除，原2项测试保留，WSL临时审查输入也已删除；结果JSON/日志继续作为验收记录，不把临时夹具写成新增常驻测试。

约21:41（Asia/Taipei）仅console由382004切到414279，实际panel PID414346、安装二进制SHA256为 `b404da3d70e90bf77f2d85984dac219f85cc29df228b4e5d388bb052ffa3d056`。构建/安装产物因CMake处理RUNPATH可有不同文件SHA；本次确认Build ID同为 `faad2f04fb5dc0cf880647bddc30cd82f0131bf9`，不能单凭两文件SHA不同断言版本漂移。

最终现场100次只读GET为100/100 matched、100/100 ready。三HTTP路径继续200，arm命令活动文件仍只有表头、零新增命令；夹爪连接/反馈正常但仍init0。arm379459、camera381998、renderer382804、recognition382810均未改变；复验中arm约100.8Hz、相机/测量约30Hz，仍为0检出窗口，motion_ready/grasp_ready未授予。此结论验证面板准入已恢复，不声称实际执行了关节或夹爪动作。

## 6 未验收项与后续维护

- 未执行实体运动、Home回程、夹爪初始化/夹持/掉落、物理断电或满棋盘识别验收；手眼内部残差不代表实体TCP精度。
- 不为健康检查自动初始化夹爪、使能机械臂、移动设备或放宽时效/匹配门限。
- 日常Home更新按[操作说明](../operations/arm-quickstart.md)采集、校验并受控加载；保留历史基准。普通安装launch需重装配置，根启动器直接读取正式源。
- 维护前读实际service、进程、参数与当前状态；不要因旧unit是transient就重建第二实例，也不要照历史PID操作。
- 本次已按CMake安装规则同步包README和手眼PORTING并核对SHA；后续文档更新也需同步安装说明，代码测试不因文档改动无条件重复。

## 7 原始证据索引

以下路径均相对RUN，不随Git克隆自动获得；应与发布源码/工件清单一并保留：

- `deployment/postdeploy-result.json`、`deployment/postdeploy-readonly.json`：service、实际进程、参数、ROS/HTTP观测及检查边界。
- `deployment/arm-cutover-result.json`：目标/基准加载、非零扭转、旧命令归档和新命令记录计数。
- `deployment/config-result.json`、`deployment/registered-units.json`、`deployment/backup/`：原入口、配置和unit变更。
- `prepush-source-checks.json`：发布源码无漂移、推送前主工作区干净、231本地文档链接和安装文档SHA。
- `test-final-results.txt`、`test-home-results.txt`、`panel-restart-diagnosis.log`：最终通过与保留的初次失败。
- `operator-rebuild.log`、`before-tools/`：独立维护工具重建/测试及旧产物。
- `live-checkpoint-compat.json`：不可变真实存档副本兼容；它不是现场恢复动作回执。

过时文档的六处移除与原文SHA见[迁移记录](documentation-migrations-20261102.md)；历史报告继续保留其冻结上下文。

面板专项证据另在P：`deployment-result.json`保存最终100/100及PID/命令计数；`temporary-fixture-cleanup.json`记录16类临时检查、执行时脚本SHA及删除/恢复注册证明；`peer-final-review.json`、`peer-acceptance.json`、`peer-source-verification.json`保存最终合规静态复审、主代理验收、源码一致及临时输入清理。初次缺陷与审查问题均保留，不用最终通过覆盖失败事实。
