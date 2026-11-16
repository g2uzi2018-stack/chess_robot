# 项目文档入口

1. **架构和接口**：[ROS 节点与接口](ros_interfaces.md)。业务数据从相机基础输出进入感知和场景，扫描/抓放通过 arm、gripper 接口调度。
2. **构建和目录**：[构建区说明](build-layout.txt)、[工具入口](../tools/README.md)。根 README 保留最短开发流程。
3. **操作**：[机械臂](operations/arm-quickstart.md)、[独立维护](operations/arm-maintenance.md)、[网页控制台](operations/web-panels.md)、[扫描抓放](operations/pick-place.md)、[演示和恢复](operations/demo-speed.md)、[运行日志](operations/logging.md)。
4. **分析与工具**：[工作空间分析](operations/workspace-analysis.md)、[业务分析](operations/arm-business-acceptance.md)、[有限往返](operations/arm-roundtrip.md)、[模型训练](vision/training.md)。
5. **数据与模型**：[稳定观察](vision/stable-observation.md)、[深度候选实验接口](vision/depth-candidates.md)、[模型工作流](models/urdf-workflow.md)、[网页模型](models/ag95-browser.md)、[绕转定义](design/rod-twist-definition.md)、[线缆策略](design/arm-cable-policy.md)。
6. **原始资料**：[厂家协议](vendor/dobot/README.md)、[Dobot 来源](../reference/dobot/README.md)、[AG95 来源](../reference/ag95/README.md)。历史实验记录移至 [runs/records](../runs/records/validation/README.md)，不作为现行操作规则追加在这里。

修改模块时，同步核对根 README、适用 AGENTS、相关 docs、模块 README、tools/README.md 和构建说明。
同一内容维护一份现行说明；节点职责、接口、启动和数据语义变化时，同次更新 ros_interfaces.md。
