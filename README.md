# 中国象棋夹取

Nova5＋AG95＋D405 的 ROS 2 Humble 工程。目标为识别、定位与夹取中国象棋棋子。

## 1 当前能力与边界

| 功能 | 已有能力 | 验证边界 |
|---|---|---|
| 机械臂 | 受限运动、单关节手动控制、连续扭转历史、停稳存档恢复 | 实机结果绑定报告中的模型、配置与路径；不是完整工作空间认证 |
| 夹爪与控制台 | AG95节点、开度/状态反馈、网页点动、六关节控制 | 单颗红卒抓取及20mm抬升获目视确认；释放、叠放及连续自动抓取尚未验收 |
| 相机与预览 | D405同步RGB-D、独立视频与检测叠加 | 相机帧率、网页刷新率与完整识别率分别统计 |
| 象棋识别 | 独立相机测量与场景状态、同帧全图候选＋ROI类别、两秒稳定快照 | 11月3日四子窗口23.63fps；不外推满棋盘或真实移动/遮挡 |
| 标定 | 48观测手眼候选外参及激光粗检 | 内部一致性不是实体绝对精度；真实夹指中点TCP和实体误差仍待量化 |

轻量YOLO26n单类候选模型已接入生产；ROI分组和batch仍为未部署实验；深度预检测只在 `depth_shadow` 实验中使用。
四子候选＋ROI约35ms是离线阶段耗时，不是完整ROS实时帧率。自动运动/抓取资格不能由检测或快照成功推导。

单颗试抓结果、候选高度与叠放前待办见[2026-11-04抓取记录](docs/validation/xiangqi-pick-trial-20261104.md)；[受监督流程入口](tools/pick_stack/README.md)执行入口先张爪并回Home，再做完整抓放前检；高度量测见[汇总记录](docs/validation/xiangqi-height-measurements-20261104.md)。

最新实机结果见[2026-11-05 裸桌排一行](docs/validation/xiangqi-row-live-20261105.md)：已独立验证摆好 1 颗；第二颗起抬失去夹持后停止，空爪已撤离并回 Home。此前两层叠放及七点激光观察的事实与限制一并记录。

## 2 从这里查阅

1. **使用和诊断**
   - [机械臂启动、Home、历史恢复与故障定位](docs/operations/arm-quickstart.md)。
   - [机器人控制台、夹爪点动与网页维护](docs/operations/web-panels.md)。
   - 本现场网页：[图像预览](http://192.168.30.26:8088)、[机器人控制台](http://192.168.30.26:8089)。
2. **开发和接口**
   - [文档与功能包索引](docs/README.md)。
   - [机械臂](src/chess_robot_arm/README.md)、[控制台](src/chess_robot_panel/README.md)、[视觉链](tools/testing/xiangqi_tracking/README.md)。
3. **版本与证据**
   - [验证索引及适用范围](docs/validation/README.md)。
   - [手眼标定](docs/validation/handeye-20261030.md)、[候选/ROI与轻量模型](docs/validation/xiangqi-precheck-20261102/README.md)。

## 3 源码与现场版本

工程位于Jetson AGX Orin的 `/home/kuang/workspace/chess_robot`，Ubuntu 22.04 / aarch64。
夹爪节点原在同一Git仓库的 `feat/gripper-node-20261025` worktree开发，不是另一套机器人；源码整合与现场驱动更新分别验收，不从原夹爪worktree的旧机械臂/视觉包覆盖主工作区的新版本。
Home目标/历史基准分离及夹爪整合已完成源码验证和本次无动作部署检查；新目标已加载，历史未清零；随后修复100Hz面板配对问题，现场100次只读准入均通过。具体运行版本、门限与未验收范围见[11月2日发布记录](docs/validation/project-release-20261102.md)，不把节点健康写成实机运动或识别准确率验收。
运行入口和安装前缀以[现场操作说明](docs/operations/arm-quickstart.md)与实际service/进程为准。提交或push不会自动切换冻结视觉源码或已运行的二进制。

## 4 资料维护

- 包README维护构建和接口，`docs/operations/`维护当前操作，`docs/validation/`保存版本明确的结果与证据。
- `runs/`包含运行安装、冻结源码、模型依赖及原始证据；被Git忽略不表示可删除。`.runtime/`保存现场配置和检查点，不随克隆分发。
- `reference/`与`docs/vendor/`保留来源、哈希、许可证和厂家协议。Xacro是模型源，URDF为生成物；模型中的厂家速度/力矩占位不等于已核实的实机限值。
- 旧[nova项目](https://github.com/g2uzi2018-stack/nova)提供过通信和开合约定参考；旧业务代码不能代替当前节点的完整回包校验、反馈确认和不自动重试机制。
- 机械臂、夹爪、相机和预览节点使用C++；当前象棋测量/场景链使用独立Python进程。Python也用于离线分析、训练编排和回归。
- 历史报告中的“当前/待部署/未提交”指当时快照。后续更新应追加新的证据，不能把旧结果改写为新版本验收。
