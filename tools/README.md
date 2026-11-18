# 项目工具

从项目根目录执行。涉及 ROS 或模型共用库的 Python 工具，先运行 `source tools/env.sh`。
工具能否连接设备、发出运动命令，以各入口参数及下列说明为准。

| 用途 | 入口 | 说明 |
| --- | --- | --- |
| 开发构建 | `bash tools/build.sh [colcon 参数]` | [构建区](../docs/build-layout.txt)，不启动节点 |
| 独立机械臂维护 | `tools/arm/arm_prepare.sh`、`arm_home.sh`、`arm_capture_home.sh` | [维护操作](../docs/operations/arm-maintenance.md)；只有显式操作选项才写设备或应用配置 |
| Home 数据核对 | `python3 tools/arm/validate_home.py --root "$PWD"` | 只读核对目标、来源与镜像，不证明实体位置 |
| 模型生成与检查 | `tools/description/generate_models.py`、`check_models.py`、`export_browser_model.py` | [模型工作流](../docs/models/urdf-workflow.md)、[网页模型](../docs/models/ag95-browser.md) |
| 工作空间离线分析 | `bash tools/analysis/run_workspace_study.sh runs/<批次>` | [分析用法和依赖](../docs/operations/workspace-analysis.md) |
| 业务运动分析 | `tools/analysis/business_node_acceptance.py` | [操作边界](../docs/operations/arm-business-acceptance.md)，不是纯离线工具 |
| 机械臂有限往返 | `tools/testing/arm_roundtrip.py` | [往返操作](../docs/operations/arm-roundtrip.md)，连接真实设备的模式需核对现场 |
| 既有回环验证 | `tools/testing/arm_ros_integration.py`、`gripper_ros_integration.py` | 使用各脚本的 localhost 夹具；不把实体节点作为夹具，不运行冒烟测试 |
| 模型训练与评估 | `tools/training/xiangqi_piece/` | [训练说明](../docs/vision/training.md) |
| 视觉离线分析与引擎生成 | `tools/vision/` | 使用对应脚本的 `--help`；历史对比需匹配原采集资料 |
| 夹取几何和标定辅助 | `tools/pick_stack/geometry_bounds.py`、`observe_contact_point.py`、`observe_vertical_step.py`、`grasp_trial.py` | 后三项可能发起实体动作；常规扫描和抓放业务见下方源码入口 |

扫描、抓放调度和设备准备在 [chess_robot_tasks](../src/chess_robot_tasks/README.md)。
感知与场景节点在 [chess_robot_perception](../src/chess_robot_perception/README.md)。
现场冻结代码仍通过 tools/description/rigid_geometry.py 的兼容符号链接读取同一份实现；不维护副本。
共用几何实现位于 `src/chess_robot_description/python/chess_robot_description/rigid_geometry.py`。

构建产物写入 `build/`；运行采样和分析结果写入 `runs/<批次>/`，长期采用的数据写入 `data/`。
新增前查现有工具，优先复用。本文是 tools 内唯一 README；修改入口时更新本文和实际调用方。
