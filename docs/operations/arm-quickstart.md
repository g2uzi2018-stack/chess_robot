# 机械臂日常启动、断电恢复与故障定位

## 1 先确认版本与状态

1. **适用范围**
   - 当前安装为 exploration，使用停稳存档；仓库默认模板与现场状态分开。
   - 2026-10-31 核对：PID **2798832**；运行前缀 `runs/feedback_window_1s_20261031_01/install`。
   - 二进制 SHA256：`cbda6f5fb8cf0ec99fff48107fbbc77d8c2baf23eab9ec6bacfab4b53d747d88`。
   - 这是核对时快照；PID 可能随后续人工启动改变，不按旧 PID 杀进程或替换安装。
2. **只读检查**
   - 在 AGX 工程根目录执行；以下命令只读取入口和状态。

   ```bash
   cd /home/kuang/workspace/chess_robot
   cat .runtime/arm-install-prefix
   source /opt/ros/humble/setup.bash
   export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1
   ros2 param get /chess_robot_arm feedback.freshness_s
   ros2 topic echo /arm/state --once
   ```

   - `feedback_fresh=true`、`reference_valid=true`、`busy=false` 与静止反馈需同时成立。
   - `diagnostic` 中 `checkpoint=durable stopped save ready` 表示存档已持久化；仅 service/process 存活不表示可恢复。
   - 1 秒是反馈的有效期，**不是采样频率**；当前无新增真实运动资格验收。

## 2 停稳、存档与重新启动

1. **断电前**
   - 等动作结束、连续停稳约 **1 秒**，确认最新干净存档就绪再断电。
   - 健康静止时每 **10 秒**周期保存；动作结束或观察到运动后的停稳会尽快保存。
   - 存档含关节、完整展开角与各段扭转、模型/Home/TCP/控制器身份和校验；不只保存净/总扭转。
2. **原位恢复条件**
   - 操作者保证断电期间没有移动；最新存档干净有效，当前关节、模型、Home、TCP、控制器身份一致。
   - 当前反馈新鲜、连续停稳，关节/扭转仍在限内，报警检查通过。
   - 关节端点默认允许 **0.005 rad**，受 **0.01 rad** 与原关节步长上限限制；端点接近本身不能证明未移动。
3. **启动已有安装**
   - 只在确认没有现存机械臂节点/控制所有者、满足上述条件后启动一个实例。
   - 当前配置允许恢复成功后检查报警并使能；启动会按配置使能，不能当作纯只读探针。

   ```bash
   cd /home/kuang/workspace/chess_robot
   bash tools/arm/run_with_checkpoint.sh
   ```

   - 入口读取 `.runtime/arm-install-prefix`，缺省才回退 `install/`；配置为 `.runtime/arm.yaml`，Home 为 `src/chess_robot_arm/config/home.yaml`。
   - 成功恢复保留扭转，不必先 Home、不必重新读取历史巨型日志，不自动运动或清错。
   - 恢复后仍核对新鲜 mode5、有效参考与停稳状态，再单独准备后续任务。

## 3 拒绝恢复与日志定位

1. **存档拒绝**
   - 运行中掉电、脏档、缺档、损坏、配置/端点失配不自动恢复，也不回退旧档。
   - 实际运动发出前先持久化不可恢复标记；原子替换与 SHA256 防止半写入被当作干净档。
   - 保留 `.runtime/arm-checkpoint.save`、运行配置和日志，按明确错误定位；不能在非Home当前位置初始化参考或清零扭转。
2. **反馈或运动故障**
   - 当前 `feedback.freshness_s=1.0`；commissioning 上限仍 **0.1 s**。
   - ServoJ ACK **0.15 s**、Stop ACK **1500 ms**、普通命令 **500 ms** 均独立，反馈读取期限覆盖1秒。
   - 重复旧帧不刷新时效；断链、非法帧、时间回退、关节/相位跳变仍拒绝。
   - `STOP_FAILED` 不能由取消响应当作已停；检查真实停稳证据及控制权，禁止自动重发失败目标。
3. **日志位置**
   - `.runtime/audit/`：异步轮转，每流 **64 MiB / 16段**（含活动段）；诊断日志不参与日常恢复。
   - `feedback.bin` 为活动段，`.1` 为上一段；队列/磁盘异常在状态和 `*.status` 报告。
   - 历史 audit 与 `runs/` 验收证据保留原字节；不要把大日志删掉来修复参考。
4. **旧客户端误拒绝**
   - 旧 `runs/` 脚本可能自检 **0.2 s**，而节点当前为 **1.0 s**；先读取节点参数，再适配新客户端。
   - 100/200 ms 报告仍属原版本结果，不能改成1秒验收；延迟根因尚未确定。

## 4 人工维护与验证边界

1. **独立维护**
   - 清错/使能、直接关节Home、采集Home统一见 [tools/arm](../../tools/arm/README.md)。
   - 这些入口绕开业务规划/防绕转；需单独确认控制权和路径，不是每次开机必跑。
   - 采集Home不修改已运行节点的内存配置；回Home不清零扭转。
2. **证据**
   - [存档机制与审查修复](../validation/checkpoint-recovery-20261031.md)、[1秒反馈验证](../validation/feedback-window-1s-20261031.md)。
   - 已测空闲进程重启与历史断电边界恢复；部署后**尚未追加真实物理断电试验**。
   - [历史大日志恢复契约与结果](../validation/reference-recovery-history.md)仅用于迁移/追溯。


## 2026-11-01：历史恢复与使能分离

停稳 mode4 可恢复和周期保存原绕缆历史，不要求先使能。建议明确使用 `checkpoint.enable_after_restore: false` 做纯历史恢复；已有 true 配置保留为显式兼容选项，失败单独报告 actuator，不再无差别抹去历史。只有新的连续可信停稳端点确认允许保留参考，反馈 gap/跳变等仍失效。详见 [历史与视觉解耦](../validation/history-vision-decouple-20261101.md)。本次修复没有现场加载 arm 二进制，也没有改变 `.runtime/arm.yaml`。
