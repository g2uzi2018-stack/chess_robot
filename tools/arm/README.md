# 独立机械臂维护工具

三个入口只调用本目录的 C++ 程序。**不依赖 ROS、不链接任何 chess_robot 业务库，
不要求启动抓取节点，也不读取或重置业务扭转参考。** 唯一共享的是 Home 数据文件。
构建依赖 C++17、CMake、yaml-cpp；GTest 用于测试。AGX 已具备这些依赖。

```bash
cd /home/kuang/workspace/chess_robot
bash tools/arm/build.sh
```

## 1. 清错误、使能及 TCP 控制

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

## 2. 独立回 Home

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

## 3. 采集 Home

```bash
# 现场示教好姿态后采集，默认只保存候选
bash tools/arm/arm_capture_home.sh
# 采集并写入正式 Home；先备份旧 YAML 和 JSON
bash tools/arm/arm_capture_home.sh --apply
```

连续采集默认 3 秒、至少 100 个唯一反馈帧，要求 mode=4/5、低速、无反馈中断且关节漂移
不超限。位置取逐轴中位数，保存反馈 CSV、候选 YAML 和来源 JSON，输出目录在 `runs`。
采集不发送任何控制器写入指令。`--apply` 仅更新本地 `config/arm/home.yaml` 及
`home_reference.json`，保留旧文件备份，不会移动机械臂或清零编码器/扭转参考。

Home YAML 和 JSON 的更新使用临时文件替换、失败回滚及 `.home_update_pending` 标记。
若掉电留下标记，先按标记路径检查备份并完成恢复；工具会拒绝继续使用不明确的 Home。
采集 Home 不等于证明线缆中性、姿态向下或回程无碰撞，需要现场确认所采姿态。

业务节点启动时加载自己的参数，采集不会偷偷修改其内存或安装目录配置。如果要让业务节点
采用新 Home，应显式更新其 `home.joint_positions_rad` 配置并重启；本维护工具立即读取正式文件。

## 配置与独立运行

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
