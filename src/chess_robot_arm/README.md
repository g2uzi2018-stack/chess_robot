# 机械臂节点：接口、规划与执行

ROS 2 Humble / C++17，`chess_robot_arm / arm_node`。日常启动与恢复统一见[操作说明](../../docs/operations/arm-quickstart.md)。

## 1 构建与配置

1. **离线构建验证**
   - 从工程根目录执行；脚本构建 twist/interfaces/arm 并运行单元测试，不启动节点、不连接控制器。

   ```bash
   bash src/chess_robot_arm/scripts/validate_target.sh
   ```

2. **配置层次**
   - `config/arm.yaml`：默认 `read_only`、回环地址、`motion.paused=true`、`hardware.enable=false`。
   - `config/home.yaml`：根 `config/arm/home.yaml` 的日常返回目标镜像，`home.joint_positions_rad`；默认已改为新采集值，不是六轴零位。
   - 根 `config/arm/twist_reference.yaml`：`reference.joint_positions_rad`，原v1扭转历史基准；构建时连同正式Home安装到包内，`arm.launch.py`加载两份。日常Home采集不修改此基准。
   - CMake检查根Home与包内镜像逐字一致；只复制包目录而漏掉根config不能作为完整构建快照。
   - `config/commissioning.yaml`：有界验收模板；反馈上限 **0.1 s**，TCP半径≤0.04 m、关节半径≤0.12 rad、净/总扭转≤20°。
   - `config/exploration.yaml`：探索模板，反馈 **1.0 s**；low/high速度选择不扩大几何范围，不报告正式认证。
   - 现场 `.runtime/arm.yaml` 与 `.runtime/arm-install-prefix` 为安装覆盖；不将模板的暂停/使能状态写成现场状态。
   - 参数启动后不可变；日常Home采集/应用后经受控重启加载，不开放通用参数热更新。更新入口与只读校验见[Home操作](../../docs/operations/arm-quickstart.md#3-更新日常home)。
   - `Config::home`仅为日常目标；`Config::twist_reference`用于Model几何基准。修改前者不改变完整展开历史或检查点身份；模型/附件或历史基准变化需重新核对，不能修改identity强行兼容。
3. **时效与超时**
   - 当前探索窗口上限 **1.0 s**；通用 hardware 默认未改，commissioning上限仍0.1 s。
   - 接收期限覆盖反馈窗口；重复帧既不刷新状态，也不重开读取期限。
   - ServoJ ACK **0.15 s**、Stop ACK **1500 ms**、普通命令 **500 ms** 独立；1秒并非反馈频率。
   - [1秒变更证据](../../docs/validation/feedback-window-1s-20261031.md)记录该批次单元/loopback与停稳加载；后续100Hz发布与一次下降记录见[验证索引](../../docs/validation/README.md)，不能合并成完整运动范围验收。

## 2 ROS 接口

### 2.1 目标与返回

1. **`/arm/move`：MoveArm Action**
   - 功能：规划或执行一个受检目标；忙时拒绝，不排队。
   - 入参：`target` 为 `base_link` 下的 `PoseStamped`，m、XYZW；感知目标保留原始非零采集时间戳，不能以发送时间替换。
   - 入参：`operation` 为 PREGRASP=0、DESCEND=1、RETREAT=2、GRASP_SEQUENCE=3、HOME=4。
   - 入参：`profile` 为空时取 `profiles.default`；`plan_only=true` 仅规划。
   - 返回：`code`、`detail`、`executed`、`final_tcp`/`final_tcp_valid`、`trajectory`；[权威定义](../chess_robot_interfaces/action/MoveArm.action)。
   - PLAN_READY=0 表示计划可用、未运动；COMPLETED=1 要求实测到位；`executed` 表示可能尝试过运动，不能当作成功。
   - 反馈：`phase`、`job_id`、TCP有效位、参考有效位及净/总扭转。
2. **目标语义**
   - PREGRASP：目标加基座+Z接近间隙，朝向不变；手眼包会核对配置后反算输入目标。
   - DESCEND/RETREAT：当前XY和姿态已对齐，按当前模型XY/姿态走竖直线，无隐藏横移/转向。
   - GRASP_SEQUENCE：先检查三段，再执行预抓→下降→WAIT_GRIPPER→撤离；不发夹爪命令。
   - `testing.home_roundtrip_precheck=true` 在零接近间隙时复制扭转历史检查目标→正式Home返程，实际返程仍为独立HOME任务，不隐式执行。
   - 规划序列轨迹不包含夹爪等待时间，不能脱离任务状态机重放；确认失败或超时结束，不自动撤离。
3. **`/arm/confirm_grasp`：ConfirmGrasp Service**
   - 功能：确认当前WAIT_GRIPPER阶段；调用者仍需独立证明夹持结果。
   - 入参：`job_id` 为该Action反馈中的精确UUID十六进制字符串，另有 `success`、`detail`。
   - 返回：`accepted`、`detail`；过期或其他任务确认拒绝。

### 2.1a 六关节手动角度

`/arm/move_joints` 使用 [MoveJoints](../chess_robot_interfaces/action/MoveJoints.action)，不会改变原有 MoveArm 定义。`positions` 与 `expected_start` 按 joint1～joint6 排列，单位 rad；`header` 保留匹配实测 JointState 的原始时间戳、frame_id 为 base_link。`profile` 与 `plan_only` 和 MoveArm 相同，结果及反馈字段也相同。

每次仅允许一个关节相对于 expected_start 改变，幅度不超过 10°，不进行角度取模。规划时必须停稳、反馈新鲜且参考有效，当前位置与 expected_start 的差异不得超过 joint_tolerance。它沿用 PTP 采样、模型关节限位、工作空间、全臂几何与线缆扭转检查，允许手动调整工具朝向，不强制抓取轴向下；模型和几何检查仍然有效。roundtrip 测试模式禁止该接口。

两个 Action 共用 TaskGate、规划/执行线程、取消处理、持久化运动标记和实测到位确认；忙时拒绝，不排队。不自动 Home、初始化、清错或重试。测试夹具接口位于 `/simulation/arm/move_joints`。

### 2.2 状态与参考

1. **测量状态**
   - `/arm/state`：[ArmState](../chess_robot_interfaces/msg/ArmState.msg)，含backend、phase/job_id、busy、feedback_fresh、reference_valid、controller_mode、TCP、扭转与diagnostic。
   - `/arm/joint_states`：`sensor_msgs/JointState`，仅新鲜真实关节位置/速度；不合成反馈。
   - `state.header`、TCP、JointState与最终TCP采用完整反馈帧收到后的ROS时刻；发布不能刷新原收帧时间。
   - TCP来自实测关节的URDF正运动学，不是独立笛卡尔传感器；无物理精度认证。
2. **`/arm/initialize_reference`**
   - 功能：显式建立已知线缆历史基准的几何参考；不清零编码器，不用于更新日常Home。
   - 入参：空；返回 `success`、`detail`。仅新鲜、连续停稳、接近 `reference.joint_positions_rad` 且空闲时允许；仍需明确真实参考可重建，接近姿态不证明无未观测绕转。
   - 到达新日常Home不允许借此初始化或清零。正常有持久历史时用恢复，不用初始化替代。
3. **`/arm/restore_checkpoint`**
   - 功能：自动恢复未开启时显式恢复最新有效停稳存档。
   - 入参：`no_unobserved_motion` 布尔声明；返回 `success`、`detail`。success 表示恢复后的历史资格，不等同执行器使能或运动就绪；可选使能失败但新鲜连续停稳确认保留历史时可返回 true，detail/actuator 诊断明确失败。
   - 需真实无未观测运动、配置/关节/完整相位匹配，不能覆盖有效参考；[日常流程](../../docs/operations/arm-quickstart.md)。
4. **`/arm/restore_reference`**
   - 功能：原始audit的完整历史迁移/取证，不作为日常启动入口。
   - 入参：有序 `audit_files` 与 `no_unobserved_motion`；返回 `success`、`detail`。
   - 从已知历史参考姿态重放完整展开相位，不能从新日常Home猜测原始零基准或缺失圈数；[历史边界契约](../../docs/validation/reference-recovery-history.md)。
   - 离线 `migrate_checkpoint_offline` 审阅计划必须显式给出 `reference_parameters`；缺项拒绝，不能从 `home_parameters` 推断历史基准。它不是任意更换历史零点的工具。
5. **回环命名空间**
   - `test_fixture=true` 必须为IPv4 `127.0.0.1`；端点改为 `/simulation/arm/...`、`simulated=true`。
   - 模拟状态不发布到真实端点，不将模拟结果当作硬件认证。

## 3 传输、规划与执行

1. **驱动与单位**
   - 固定官方C++ V4 SDK编码FK/IK（正/逆运动学）、ServoJ/Stop、解码1440字节 `CFeedbackData`；来源在 `third_party/dobot_v4/UPSTREAM.json` 与 `LOCAL.patch`。
   - SDK只用user=0/tool=0（基座/法兰）；内部m/rad，SDK边界mm/deg及 `Rz*Ry*Rx`；TCP转换在本地完成。
   - 线缆数字使用十进制，避免控制器拒绝接近零时的科学计数法。
2. **单连接与有界确认**
   - 同一主机只用一条有序命令连接；每次核对错误码、原命令回显与整次期限，保留不完整notice诊断。
   - ServoJ异步FIFO默认最多6条，严格回显及递增队列ID；125ms ACK可与30ms发点周期并存，超时/溢出仍失败。
   - 含糊回包后禁止后续查询/运动，不自动重连重放；保留原连接只尝试一次Stop并核对自身ACK。
   - 停止预算含在途查询/连接I/O、ACK和实测停稳；物理断链不能保证Stop成功，明确返回STOP_FAILED。
   - 反馈校验长度、标记、有限字段及官方结构offset；相同控制器时间戳丢弃，回退使参考失效。
3. **扭转与几何**
   - 实测反馈独占推进 `chess_robot_twist::Tracker`；规划复制完整历史，不做关节绕回/圈数猜测。
   - 每个规划/执行采样检查关节界、相位步长、工作区、净/总扭转和几何；无连续碰撞保证。
   - GeometryGuard由 `tools/analysis/workspace_analysis.py --probe-only` 导出；来源见[离线工作区报告](../../docs/analysis/arm-workspace-2026-10-24.md)。桌面z=-0.010m，活动件20mm余量，非相邻组10mm，关节余量5°，归一化Jacobian最小奇异值≥0.05、条件数≤80。
   - 固定/工具组与直接相邻组有明确排除；21开度夹爪包络不等于完整场景。
4. **选解与采样**
   - 厂家当前/Home seed的候选都核对SDK FK(IK)、URDF FK和路径；总扭转→净扭转绝对值→耗时排序。
   - 5°中性带、2°角度滞回、1s耗时滞回抑制数值抖动；旧cost权重兼容解析但不参与排序，不证明全逆解全局最优。
   - 竖直IK步长2mm、分支关节步长≤0.025rad；C1 Hermite与单一五次时间律按30ms采样，复查速度/加速度后必要时拉长时间。
   - 重新检查竖直偏差、姿态、扭转和几何；离散模型不保证样本间连续极值或实物精确轨迹。
5. **完成与取消**
   - ACK不等于到位；最终关节/TCP误差、低速、mode5与连续dwell均需实测满足。
   - 取消/故障/退出只有拥有运动的任务才Stop；只读/仅规划不发送Stop。
   - Stop ACK或停稳验证失败锁定FAULT并拒绝新目标；不能通过提交Home清故障。
   - 节点任务/反馈线程及驱动回复线程均join，无detached线程；关机验证期间反馈持续工作。

## 4 存档、诊断与适用范围

1. **停稳存档**
   - 保存周期10s、连续停稳1s；持久化完整局部展开相位、关节及配置指纹，checksum/fsync/原子rename/目录fsync，单写者锁。
   - schema1身份字节布局保持；原Home槽位绑定同一个历史基准。日常目标变化不改identity，旧存档的关节、previous_phase和local保持；历史基准、模型、TCP或控制器变化仍拒绝。
   - 实际运动前先写脏标记；坏/脏/不兼容/端点失配拒绝恢复，不回退旧档，不在非Home造初始参考。
   - `checkpoint.auto_restore` 要求 `checkpoint.no_unobserved_motion`；`enable_after_restore` 才在成功后GetErrorID并按配置EnableRobot，不ClearError、不自动运动。
   - 可选使能失败单独报告；仅后续新的连续可信停稳端点确认可保留已提交历史，反馈gap、跳变或确认不可信仍使对应参考失效。当前进程不自动反复使能；详见[恢复状态机](../../docs/validation/history-vision-decouple-20261101.md)。
2. **日志**
   - `audit.directory` 可选；独立异步队列8MiB/8192记录，默认每流64MiB/16文件含活动段，二进制记录不跨段。
   - 原始记录为int64主机UTC纳秒＋1440控制器字节，小端；队列/磁盘错误报告丢弃数，不阻塞反馈。
   - `diagnostics.feedback_trace` 默认false；启用时预分配262144行，反馈线程退出后写timing CSV/meta，满额有drop计数。
3. **验收边界**
   - 默认low=0.708rad/s、1.416rad/s²；high=2.36rad/s、4.72rad/s²来自固定官方ROS规划配置，不是现场控制器限值认证。
   - 现场另用roundtrip_low=0.3、roundtrip_high=1.0、alignment=0.4的速度/加速度规划上限；以 `.runtime/arm.yaml` 为准。
   - `backend=hardware` 要求hardware下五项布尔均true；它们是操作者声明，默认false。exploration/commissioning不会提升为认证模式。
   - 标称TCP、实际控制器限值、附件模型、完整碰撞场景、ServoJ/Stop实测范围仍需按任务核对；外部控制客户端不受本节点排他锁保护。
   - [业务探索操作](../../docs/operations/arm-business-acceptance.md)、[验证索引](../../docs/validation/README.md)。

## 2026-11-01：ROS状态发布频率

`feedback.publish_hz` 默认20Hz，允许1～125Hz，启动后不可变。现场本轮已验证100Hz并写入`.runtime/arm.yaml`。该参数只调整ArmState/JointState及Action反馈发布；原始控制器采样、运动时序、安全新鲜度门限不变，停稳存档/任务维护仍为20Hz。重复发布不刷新源反馈stamp；应分别统计消息频率与唯一反馈时间戳频率。原始30004流本轮约122Hz，不宣称其硬件上限可由本参数提高。
