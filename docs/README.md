# 项目文档索引

## 1 架构与职责

1. **数据与控制链**
   - D405 → `vision_node`：采集、彩深对齐、校正、内参与帧信息。
   - 图像 → `preview_node` → 浏览器：按需订阅、缩放、深度伪彩色与 JPEG；不做棋子检测。
   - RGB-D → `xiangqi_perception` → 相机测量JSON → `xiangqi_scene`：后者维护基座变换、跨帧ID和两秒快照；缺机械臂时相机检测可继续。
   - `tracking_preview`：独立叠加绘制；视频帧率不代表识别完成率。
   - xiangqi_scan_controller：按需编排Home→17个停点→Home，复用arm Action与scene；[运行说明](../tools/scan_table/README.md)、[限定验证](validation/xiangqi-table-scan-20261103.md)。
   - 目标 → `/arm/move` → Planner → FK/IK、扭转与几何检查 → Executor → Nova5 ServoJ。
   - 实测关节 → `chess_robot_twist` 连续历史 → `/arm/state` 与停稳存档。
   - `handeye_node` 使用现有运动与视觉接口，完成停稳采样、PARK 求解和留出验证；不发布生产 TF。
   - `panel_node` 设备切换 → `/arm/move_joints` 或 `/gripper/command` → 机械臂或夹爪驱动；尚未接成自动夹取闭环。
2. **单位与参考系**
   - 工程长度 **m**、角度 **rad**、四元数 **XYZW**；厂家 SDK 边界才转换 mm/deg。
   - `base_link` 为基座，`Link6` 为法兰，`d405_color_optical_frame` 为相机光学系（X右/Y下/Z前）。
   - 标称 TCP 偏移 **0.20 m**，真实夹指中点仍待标定；生产手眼 TF 尚未发布。

## 2 构建、配置与接口

1. **功能包**

   | 包 | 责任与文档 | 主要配置 |
   |---|---|---|
   | `chess_robot_arm` | [构建、Action/Service、执行保护](../src/chess_robot_arm/README.md) | [arm](../src/chess_robot_arm/config/arm.yaml)、[exploration](../src/chess_robot_arm/config/exploration.yaml)、[日常Home](../config/arm/home.yaml)、[历史基准](../config/arm/twist_reference.yaml) |
   | `chess_robot_twist` | [全臂几何扭转库](../src/chess_robot_twist/README.md)；无硬件 I/O | [扭转定义](design/rod-twist-definition.md)、[线缆策略](design/arm-cable-policy.md) |
   | `chess_robot_gripper` | [AG95动作、反馈与故障语义](../src/chess_robot_gripper/README.md) | mock默认、hardware只读模板 |
   | `chess_robot_panel` | [HTTP、网页、关节/夹爪互斥](../src/chess_robot_panel/README.md) | [panel](../src/chess_robot_panel/config/panel.yaml) |
   | `chess_robot_interfaces` | [MoveArm](../src/chess_robot_interfaces/action/MoveArm.action)、[ArmState](../src/chess_robot_interfaces/msg/ArmState.msg)、[恢复服务](../src/chess_robot_interfaces/srv/RestoreCheckpoint.srv) | 消息为接口权威定义 |
   | `chess_robot_vision` | [构建、图像与时间语义](../src/chess_robot_vision/README.md) | [D405](../src/chess_robot_vision/config/d405.yaml) |
   | `chess_robot_preview` | [构建、HTTP与通道扩展](../src/chess_robot_preview/README.md) | [channels](../src/chess_robot_preview/config/channels.yaml) |
   | `chess_robot_tracking_preview` | [独立叠加与过期清除](../src/chess_robot_tracking_preview/README.md) | 包内配置 |
   | `chess_robot_handeye` | [构建、配置与采样](../src/chess_robot_handeye/README.md)、[移植范围](../src/chess_robot_handeye/PORTING.md) | [模板](../src/chess_robot_handeye/config/handeye.example.yaml) |
   | `chess_robot_validation` | [早期只读/模拟工具](../src/chess_robot_validation/README.md)；不是当前业务启动入口 | 包内 config |
2. **安装边界**
   - ROS 2 Humble；各包 `package.xml`/`CMakeLists.txt` 定义依赖；依包 README 构建，使用 `--base-paths src` 或明确包目录，避免扫描 `runs/` 的环境。
   - 仓库 `arm.yaml` 默认只读、暂停；现场 `.runtime/arm.yaml` 是独立安装配置，当前为 exploration。
   - `.runtime/arm-install-prefix` 选择已测试的运行版本；构建产物不能自动替换它。
   - 日常Home与历史基准分开；采集/应用只更新目标和镜像，受控重启加载新目标，不改原绕转历史。`validate_home.py`在启动前只读校验一致性；详细语义见[Home操作](operations/arm-quickstart.md#2-日常home与历史参考分开保存)。
   - 夹爪节点原在同仓库worktree `/home/kuang/workspace/chess_robot_gripper` 的 `feat/gripper-node-20261025` 分支开发；不是无关外部工程。整合后的包和统一接口以当前源码为准，现场安装仍需单独核对。
   - 控制台与夹爪组合launch会启动一个夹爪驱动；已有驱动时仅启动panel，避免重复占用硬件。运行入口见[网页操作](operations/web-panels.md)。
   - 象棋Python测量/场景链的依赖、模型与冻结生产入口见[视觉说明](../tools/testing/xiangqi_tracking/README.md)；源码默认行为不等于现场工件配置。

## 3 操作与诊断

1. **现场操作**
   - [机械臂日常启动、干净存档、断电恢复、故障定位](operations/arm-quickstart.md)。
   - [机器人控制台、六关节与夹爪点动](operations/web-panels.md)。
   - [独立人工维护工具：清错/使能、Home、采集Home](../tools/arm/README.md)；与业务启动分开。
2. **专用验收工具**
   - [业务探索流程与边界](operations/arm-business-acceptance.md)、[离线分析与证据工具](../tools/analysis/README.md)。
   - [回环夹具](../tools/testing/README.md)、[有限Home往返工具](../tools/testing/arm_roundtrip_README.md)。
   - 旧工具可能自建节点、硬编码旧时效或重放历史；复用前核对源码和当前参数，不对着活跃节点直接运行旧批次。

## 4 模型、资料与结果

1. **模型**
   - [Nova5/组合模型](../src/chess_robot_description/README.md)、[AG95资源](../src/dh_ag95_description/README.md)。
   - [模型生成、导出与在线查看](models/ag95-browser.md)、[组装/检查工作流与接力规则](models/urdf-workflow.md)。
   - [离线工作区分析及适用边界](analysis/arm-workspace-2026-10-24.md)。
2. **证据与原件**
   - [验证索引](validation/README.md)：当前结论、版本、历史报告与原始数据位置。
   - [厂家协议](vendor/dobot/README.md)、[Dobot来源](../reference/dobot/README.md)、[AG95来源](../reference/ag95/README.md)。
   - 原始标定数据、厂商手册、许可证和控制器日志不能按重复叙述报告处理。
