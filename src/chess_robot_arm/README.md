# 机械臂节点

`chess_robot_arm` 提供 Nova5 机械臂的规划、执行、实测状态和线缆扭转历史管理。
它接收运动目标，检查轨迹并通过控制器执行；不负责识别棋子或直接操作夹爪。
ROS 2 Humble / C++17，节点名为 `/chess_robot_arm`，可执行程序为 `arm_node`。

## 1 结构与修改入口

| 位置 | 用途 |
| --- | --- |
| `config/` | 节点参数、Home 镜像和直接使用的几何数据，见 [配置提示](config/README.txt) |
| `launch/arm.launch.py` | 机械臂独立启动入口，默认只读 |
| `src/node.cpp` | ROS 接口、任务调度、状态发布及存档维护 |
| `src/core.cpp` | 状态与历史、规划、任务管理、执行 |
| `src/driver.cpp` | 控制器 V4 通信、命令/反馈处理及单位转换 |
| `src/geometry_guard.cpp` | 基于 URDF 和几何数据的距离、限位与姿态检查 |
| `src/checkpoint.cpp`、`src/rotating_audit.cpp` | 历史存档、异步记录与文件轮换 |
| `include/` | 可供外部调用的 C++ 核心库接口，见 [接口提示](include/README.txt) |
| `third_party/dobot_v4/` | 厂家 SDK、许可证、固定来源和本地补丁 |
| `test/` | 既有检查与离线工具，使用边界见 [提示](test/README.txt) |

ROS 回调处理请求；一个任务线程负责规划和执行，一个反馈线程持续更新实测状态。
连续发点期间由独立回复线程读取确认；配置记录目录后，记录线程负责写盘。
`Planner`、`Executor` 是同一任务线程中的不同职责，不各自创建线程。

## 2 构建与独立启动

开发环境在 Jetson AGX Orin，Ubuntu 22.04。核心依赖包括 Eigen、FCL、yaml-cpp、OpenSSL，
以及本项目的 twist、interface、description 包；具体声明见 `package.xml`。
厂家 SDK 随包保存，来源见 [UPSTREAM.json](third_party/dobot_v4/UPSTREAM.json)。

从项目根目录构建；此命令不运行测试、不连接设备：

```bash
bash tools/build.sh --packages-up-to chess_robot_arm --cmake-args -DBUILD_TESTING=OFF
source tools/env.sh
```

构建产物在 `build/ros/`，安装在 `install/dev/`。
基础只读入口如下，将地址替换为实际控制器 IPv4；避免与已有服务重复占用设备连接：

```bash
ros2 launch chess_robot_arm arm.launch.py controller_ip:=<机械臂IPv4>
```

默认只接收反馈和允许请求仅规划，不自动建立历史参考、使能或运动。
连接正常后可读取 `/arm/state` 和 `/arm/joint_states`；反馈新鲜、参考有效、已使能是不同状态。
仅规划也需要真实新鲜反馈和有效历史；正逆运动学查询会访问控制器，不是纯离线计算。

Ctrl+C 结束独立启动的进程。只有正在负责运动的任务会尝试 Stop 并核对实测停稳；
退出消息或取消回执不能单独证明机械臂已经停下。

现场启停、Home 采集和恢复统一见 [机械臂操作说明](../../docs/operations/arm-quickstart.md)。
开发工作区的构建不会自动切换现场服务；以服务实际入口和安装目录为准。

## 3 配置怎么加载

`arm.launch.py` 依次加载：
`config/arm.yaml` → `home.yaml` → `twist_reference.yaml` → launch 指定的 IP、URDF、几何模型路径。
可覆盖的 launch 参数是 `controller_ip`、`urdf_file`、`safety_model_file`。
参数仅在启动时读取，改配置后需要让运行入口加载新文件并重启。

`commissioning.yaml` 和 `exploration.yaml` 是真实设备调试覆盖文件，默认不会自动加载。
独立加载覆盖文件时，可从项目根按下面顺序启动；将所有占位路径换成实际绝对路径：

```bash
ros2 run chess_robot_arm arm_node --ros-args \
  --params-file src/chess_robot_arm/config/arm.yaml \
  --params-file config/arm/home.yaml \
  --params-file config/arm/twist_reference.yaml \
  --params-file <覆盖YAML的绝对路径> \
  -p controller.ip:=<机械臂IPv4> \
  -p urdf_file:=<整机URDF的绝对路径> \
  -p safety.model_file:=<几何YAML的绝对路径>
```

这两份模板仍保留执行关闭和基础配置的暂停状态；仅加载文件不会自动运动。
现场恢复入口 `tools/arm/run_with_checkpoint.sh` 使用现有部署选择，
按 `.runtime/arm.yaml` → 根 Home → 根历史参考加载，不等同于上面的开发 launch。

日常 Home 是返回目标，历史参考是线缆扭转计算的基准，两者不能互相替代。
Home 镜像和大型几何文件受既有一致性/哈希约束，字段说明与修改入口见 [config/README.txt](config/README.txt)。

## 4 接口与任务行为

完整名称、类型、上下游见 [ROS 接口说明](../../docs/ros_interfaces.md)；
字段定义集中在 [chess_robot_interface](../chess_robot_interface/README.txt)。

- `/arm/move`：规划或执行位姿目标，支持预抓、下降、撤离、抓取阶段序列及 Home。
- `/arm/move_joints`：手动关节目标；相对请求中的实测起点只允许一个关节变化，幅度不超过 10°。
- `/arm/state`、`/arm/joint_states`：实测状态、任务阶段与历史资格。
- `/arm/confirm_grasp`：确认当前等待夹持的任务；调用方负责实际操作夹爪并判断结果。
- `/arm/initialize_reference`、`/arm/restore_checkpoint`、`/arm/restore_reference`：历史参考建立与恢复。

两个运动 Action 共用一个任务位置，忙时拒绝、不排队。目标采用 `base_link`，内部单位 m/rad；
来自感知的目标保留原始采集时间戳。状态时间戳来自完整反馈收帧时刻，重复发布不会刷新它。
`feedback.publish_hz` 的代码默认值为 20 Hz，影响 ROS 状态发布，不改变控制器采样或发点周期。

预抓在目标的基座 +Z 方向增加接近间隙；下降/撤离要求 XY 和姿态已经对齐，不隐含横移。
抓取序列先规划各段，再执行预抓、下降、等待夹持确认、撤离；等待失败不会自动撤离。
结果中的轨迹不包含夹爪等待时间，不能脱离任务阶段直接重放。
可选 `testing.home_roundtrip_precheck` 只预查返程，不自动执行 Home。

`PLAN_READY` 表示规划完成但未运动；`COMPLETED` 要求实测到位。
`executed` 表示可能尝试过运动，不能当作成功。命令回复也不等于已到位。
取消/故障时，只有实际拥有运动的任务尝试 Stop；Stop 或停稳确认失败会保留故障状态。

## 5 存档、记录与边界

- `checkpoint.file` 非空时保存完整线缆历史，默认停稳 1 秒、静止周期 10 秒。
  运动前先写旧档不可恢复标记；正常停稳后再保存。坏档、脏档或身份不匹配时不会回退旧档。
- 历史恢复要求真实确认未发生遗漏运动，并核对模型、工具、控制器、关节和历史端点。
  日常 Home 到位不代表线缆已解绕，不能借初始化接口清零。
  可选恢复后使能的结果单独报告；历史有效不等于已经使能。
- `audit.directory` 非空时异步记录命令和原始反馈；默认每流单文件 64 MiB、共 16 文件。
  原始反馈每条为主机 UTC 纳秒 int64 加 1440 字节控制器数据，小端。
  队列/磁盘错误报告丢弃情况，不在反馈线程同步等待写盘。
- `diagnostics.feedback_trace` 默认关闭；开启后先记入有界内存，反馈线程结束后输出时序文件。
  普通运行日志位置与异常汇总方式见 [日志说明](../../docs/operations/logging.md)。
- 规划与执行检查的是离散采样及当前模型；不能据此宣称采样之间、真实附件或完整现场均无碰撞。
  TCP 来自关节与 URDF 计算，不是独立位置传感器；名义模型不等于实测精度认证。
- 净/总扭转优先于耗时选解；连续实测反馈推进真实历史，规划使用历史副本，不猜测缺失圈数。
  当前 SDK 通信固定 user=0/tool=0，工具变换在本地完成。
- 当前仍有 `test_fixture` 和旧模拟检查实现，不属于真实设备调试入口；保持关闭，不运行模拟替身。
  纯计算检查、旧脚本和离线工具的边界见 [test/README.txt](test/README.txt)，不默认调用整套历史测试。

恢复的详细输入与行为见 [机械臂操作说明](../../docs/operations/arm-quickstart.md)。
仅阅读、整理注释或修改无关节点，不触发本包测试、模型重新生成或设备运行。
