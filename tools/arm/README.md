# 独立机械臂维护工具

三个入口只调用本目录的 C++ 程序。**不依赖 ROS、不链接任何 chess_robot 业务库，
不要求启动抓取节点，也不读取或重置业务扭转参考。** 共享的是日常Home目标数据文件；扭转历史基准与检查点由业务节点独立维护，维护采集不修改它们。
构建依赖 C++17、CMake、yaml-cpp；GTest 用于测试。AGX 已具备这些依赖。升级维护工具源码后须重建下列独立二进制，ROS的colcon构建不会自动更新它；脚本固定调用 `runs/arm_tools_build/arm_operator`。

```bash
cd /home/kuang/workspace/chess_robot
bash tools/arm/build.sh
```


## 1 入口边界

1. **日常业务节点**
   - 停稳存档启动统一用 `bash tools/arm/run_with_checkpoint.sh`，操作条件见[日常说明](../../docs/operations/arm-quickstart.md)。
2. **独立人工维护**
   - 以下三个维护工具不依赖ROS；直接回Home绕开业务规划/扭转检查，不能作为每次开机必跑步骤。


## 2 清错误、使能及 TCP 控制

```bash
# 只检查，不发送清错/使能/模式切换命令
bash tools/arm/arm_prepare.sh
# 清错并使能；当前不是 TCP 时，在厂家允许的状态下尝试切换
bash tools/arm/arm_prepare.sh --request-tcp --apply
```

顺序：检查反馈和静止状态 → 必要时 RequestControl → ClearError → 检查错误已清除 →
必要时 EnableRobot → 确认 mode=5 且静止。EnableRobot 不携带负载参数，沿用控制器设置。

厂家 V4 协议规定 RequestControl 仅能在未上电或未使能等状态使用；已使能但不是 TCP 时，
需要先在示教器下使能或手动切换。工具不会自行下使能、解除暂停、松抱闸、PowerOn 或复位急停。
没有找到独立“在线模式”命令，因此不承诺切换 DobotStudio 的 UI 在线状态。
清不掉的错误会报告失败，不会继续使能。TCP 未切好时保留 `Control Mode Is Not Tcp` 诊断。

## 3 独立回 Home

```bash
# 查看将使用的六轴角度和 MovJ 参数，无运动
bash tools/arm/arm_home.sh
# 现场确认回程路径、控制器已使能并处于 TCP 后执行
bash tools/arm/arm_home.sh --execute
```

直接发送 `MovJ(joint={...},user=0,tool=0,a=5,v=5,cp=0)`，不用抓取节点、ROS Action、
业务规划器或防绕转模块。默认速度/加速度是控制器最大值的 5% 指令比例，可在配置里调整，
不是绝对 rad/s 上限。Home 是保存的六轴示教角度，不是六轴全零。

完成必须同时满足：返回队列 ID 与 GetCurrentCommandID 相同、反馈 mode=5、
六轴实测误差足够小、速度足够低并持续停留。ACK 不等于到位。
Ctrl+C、超时、运动反馈故障或不明确的 ACK，会发送 Stop 并尝试验证已停止；
无法确认时明确返回 STOP_FAILED。不会自动清错、使能或修改速度全局参数。

这是人工维护的直接关节运动，会绕开业务路径/防绕转检查。使用前确认回程路径和控制权，
业务节点不能同时发送运动。工具之间有进程锁；外部控制客户端不受此锁保护。

## 4 采集 Home

```bash
# 现场示教好姿态后采集，默认只保存候选
bash tools/arm/arm_capture_home.sh
# 采集并写入正式 Home；先备份旧 YAML 和 JSON
bash tools/arm/arm_capture_home.sh --apply
```

连续采集默认 3 秒、至少 100 个唯一反馈帧，要求 mode=4/5、低速、无反馈中断且关节漂移
不超限。位置取逐轴中位数，保存反馈 CSV、候选 YAML 和来源 JSON，输出目录在 `runs`。
采集不发送任何控制器写入指令。`--apply` 仅更新本地 `config/arm/home.yaml` 及
`home_reference.json`和包内 `src/chess_robot_arm/config/home.yaml` 镜像，保留三份旧文件备份，不会移动机械臂或清零编码器/扭转参考；不写 `config/arm/twist_reference.yaml`。

正式Home、来源JSON与包内镜像的更新使用临时文件替换、fsync、各文件独立回滚及 `.home_update_pending` 标记；回滚不完整时保留标记并拒绝继续使用。`--apply`只接受项目正式路径与完整镜像/历史基准布局，不应用到安装目录副本。
若掉电留下标记，先按标记路径检查备份并完成恢复；工具会拒绝继续使用不明确的 Home。
采集 Home 不等于证明线缆中性、姿态向下或回程无碰撞，需要现场确认所采姿态。

业务节点启动时加载自己的参数，采集不偷偷修改其内存或安装目录配置。日常目标为 `home.joint_positions_rad`，历史基准为 `reference.joint_positions_rad`；更新前者不清零历史，后者不得由日常采集工具修改。

```bash
python3 tools/arm/validate_home.py --root /home/kuang/workspace/chess_robot
```

验证器只读检查来源类型/帧数与目标一致、六轴值/顺序/限位、镜像漂移与中断标记，不连接控制器，也不证明CSV真实性或物理中性。更新后使用受控启动入口加载根正式源；普通安装目录launch则需先重新安装配置。完整流程见[日常操作](../../docs/operations/arm-quickstart.md)。
采集工具虽不发送写命令，仍直接读取控制器反馈；维护前核对反馈连接所有权，避免与活跃实例争用。

## 5 配置与独立运行

`config/operator.yaml` 包含控制器网络、超时、关节范围、Home 比例速度/加速度和采集容差。
位置限位来自现有 Nova5 URDF；使用其他型号前必须替换配置。所有工具支持网络和文件路径覆盖：

```bash
bash tools/arm/arm_home.sh --ip 192.168.30.234 --home-file /absolute/home.yaml
bash tools/arm/arm_capture_home.sh --output-dir /absolute/new_capture_dir
```

独立二进制也可直接运行，便于部署到没有 ROS 的机器：

```bash
runs/arm_tools_build/arm_operator --help
runs/arm_tools_build/arm_operator home --config tools/arm/config/operator.yaml \
  --home-file config/arm/home.yaml
```

传输和反馈格式代码从本工程既有实现适配为本目录内部文件，不产生运行/链接依赖。
本次测试只在 localhost 控制器夹具执行写入和运动命令；未进行任何实机清错、使能、切模式或运动。

端到端协议测试（开发环境另需 Python3 和 PyYAML；执行命令全部发往 localhost）：

```bash
python3 tools/arm/test/test_loopback.py runs/arm_tools_build/arm_operator \
  tools/arm/config/operator.yaml config/arm/home.yaml
```
