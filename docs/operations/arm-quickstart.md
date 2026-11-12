# 机械臂日常启动、Home更新、历史恢复与故障定位

## 1 先核对实际安装与状态

源码、安装和正在运行的进程是三个版本层次。提交或push不会替换已运行二进制；旧报告的PID、SHA与安装前缀仅是冻结证据。
本现场默认使用ROS domain90与localhost模式，检查时先读当前service和安装选择：

```bash
cd /home/kuang/workspace/chess_robot
cat .runtime/arm-install-prefix
systemctl --user show chess_robot_arm.service --property=ActiveState,SubState,MainPID,ExecStart
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1
ros2 param get /chess_robot_arm feedback.freshness_s
ros2 param get /chess_robot_arm feedback.publish_hz
ros2 topic echo /arm/state --once
```

反馈新鲜、参考有效、非模拟、无活动任务与停稳是不同条件，不能只看进程存活。
`checkpoint=durable stopped save ready`表示该次干净存档完成；执行器状态另看mode/actuator诊断，参考有效不等于已使能或可运动。
1.0s是反馈有效窗口，100Hz是可配置的ROS发布频率，两者均不是控制器原始采样率。发布不能刷新旧源时间戳。

## 2 日常Home与历史参考分开保存

| 内容 | 配置与含义 | 更新规则 |
|---|---|---|
| 日常返回目标 | `config/arm/home.yaml`，`home.joint_positions_rad` | 已选用新采集姿态；更新目标不清零历史 |
| 采集来源 | `config/arm/home_reference.json` | 来源类型、帧数与角度须和目标匹配；原始采集数据另存 |
| 包内镜像 | `src/chess_robot_arm/config/home.yaml` | 由正式采集工具同步；构建校验逐字一致并安装根正式源 |
| 扭转历史基准 | `config/arm/twist_reference.yaml`，`reference.joint_positions_rad` | 保留原v1历史参考；日常Home采集不修改它 |

返回目标用于HOME规划、候选seed和相对Home的探索包络；历史基准用于完整展开相位、初始化、原始audit重放和检查点身份。
旧参考数值继续作为历史坐标基准，不再作为日常返回目标。改变返回目标没有让线缆在物理上解绕，因此不能把扭转归零。

同一历史基准下更新日常目标不需要迁移或重写检查点，只需受控重新加载新目标。
旧schema1检查点保留原身份字节布局：原Home槽位现在明确表示同一个历史基准。日常目标改变时身份稳定，原关节、`previous_phase`与`local`可保持；模型、TCP、控制器或历史基准变化仍拒绝恢复。
这不是任意检查点重标工具。真正更换历史基准需另立物理校准与历史重建流程，本次没有开放绕过身份校验或任意清零接口。

## 3 更新日常Home

维护采集程序是独立CMake构建，不随ROS全包构建自动更新。升级源码后先重建 `runs/arm_tools_build/arm_operator`，并核对使用的是本版本工具：

```bash
bash tools/arm/build.sh
```

1. 在维护窗口先核对设备与反馈连接所有权，停止安排新任务。维护采集工具直接读取控制器反馈；不发送写命令不代表可以与活跃连接争用。
2. 在确认的新目标姿态停稳后采集。默认只保存候选；`--apply`才更新项目正式源：

   ```bash
   bash tools/arm/arm_capture_home.sh --output-dir runs/new_home_candidate
   # 仅在确认此次采集目标后使用；输出目录必须为新的目录
   bash tools/arm/arm_capture_home.sh --output-dir runs/new_home_apply --apply
   python3 tools/arm/validate_home.py --root /home/kuang/workspace/chess_robot
   ```

3. `--apply`备份并同步根Home YAML、来源JSON和包内镜像，使用持久化中断标记、fsync与各文件独立回滚；回滚不完整时保留pending并拒绝继续使用。不移动机械臂、不使能、不修改历史基准或检查点。
4. 验证器只读检查来源字段/帧数和目标一致、六轴顺序/有限值/限位、镜像漂移及`.home_update_pending`。它不证明原始CSV真实性、路径无碰撞或线缆中性，采集原始数据仍需保留。
5. 运行中参数不热更新。经受控重启加载新目标；如果使用下面的根配置启动器，无需为单纯目标变更重新编译C++。若用安装目录的普通`arm.launch.py`，先重新安装配置，不能只重启旧安装副本。

有中断标记、来源缺失/角度不匹配或镜像漂移时，先检查采集备份并恢复一致性，不删除标记掩盖未完成更新，也不直接编辑检查点identity。
更新日常Home不调用`initialize_reference`。该服务只能在历史参考姿态且明确可重建真实参考时使用；位置接近本身不能证明没有未观测整圈。

## 4 停稳存档与受控启动

健康、有参考、空闲的mode4/5持续停稳约1s后可保存完整展开相位；静止周期约10s，动作/观察运动后的停稳尽快保存。
断电前确认最近干净存档已经持久化；动作发送前先持久化dirty标记，运动中掉电、坏档、脏档或缺历史不自动放行。

原位恢复还要求操作者真实确认断电期间没有移动、模型/TCP/历史基准/控制器身份一致，且新鲜连续停稳端点匹配。
端点默认容差0.005rad，受0.01rad与原关节步长上限约束；端点接近不能代替无未观测运动声明。

确认没有其他控制所有者，且当前启动策略符合本次维护目的后，才启动一个实例：

```bash
cd /home/kuang/workspace/chess_robot
bash tools/arm/run_with_checkpoint.sh
```

启动器先验证Home一致性，读取`.runtime/arm-install-prefix`（缺省回退`install/`），再按`.runtime/arm.yaml`→根Home→根历史基准的顺序加载，后两份防止旧runtime中的目标/基准覆盖正式源。
安装配置文件与进程实际参数仍需核对；不要从旧`runs`候选目录重装当前节点。

`checkpoint.enable_after_restore=false`为纯历史恢复，不发GetErrorID/EnableRobot；mode4可保存/恢复历史，不要求先使能。
若显式配置为true，启动可能检查报警并发送使能，因此不能当作只读探针。使能失败单独报告；仅新的连续可信停稳端点确认可保留历史，gap、跳变或不可信确认仍失效，不自动反复重试。
恢复历史不等于运动就绪；后续动作仍须满足原来的反馈、mode、几何、扭转与任务准入条件。

## 5 故障定位与维护边界

- 保留`.runtime/arm-checkpoint.save`、现场配置和诊断。坏/脏/身份或端点失配不回退旧档，不在任意当前位置清零参考。
- exploration反馈上限1.0s，commissioning仍0.1s；ServoJ ACK0.15s、普通命令500ms、Stop ACK1500ms独立。重复旧帧不刷新时效，断链/非法帧/时间回退/相位跳变仍拒绝。
- `.runtime/audit/`每流默认64MiB/16段（含活动段），独立异步诊断；文件自然增长或checkpoint周期变化不表示有人操作设备。
- `STOP_FAILED`不能由取消响应当作已停。夹爪没有已验证的独立停止接口，退出/超时也不保证物理停止。
- 清错、使能、直接MovJ Home与采集工具见[维护说明](../../tools/arm/README.md)。直接Home绕开业务路径/防绕转检查，不是日常恢复步骤。
- 旧客户端可能自检0.1/0.2s或自建节点；读取当前参数并核对所有权后再适配，不改旧报告冒称当前资格。

## 6 证据如何使用

[验证索引](../validation/README.md)区分原1秒窗口、后续100Hz/下降记录、历史恢复及本次版本。
[恢复状态机与视觉解耦](../validation/history-vision-decouple-20261101.md)和[历史audit恢复契约](../validation/reference-recovery-history.md)保留原版本范围；其中“当时未加载”的陈述不能替代当前进程核对。
实体掉电、真实运动范围、TCP精度和自动夹取须另有相应实测；离线测试、部署成功或网页GET成功不能替代它们。
