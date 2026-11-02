# 中国象棋夹取

Nova5＋AG95＋D405 的 ROS 2 Humble / C++ 工程。目标为识别、定位与夹取中国象棋棋子。

## 1 当前进度

1. **已具备**
   - 受限机械臂运动、完整扭转历史跟踪、停稳存档恢复、相机采集与网页预览、手眼候选外参。
   - 夹爪与手动面板位于独立协作工作区；尚未合入主工程。
2. **尚待完成**
   - 真实夹指中点 TCP（工具中心点）、量化实体误差、棋子识别与自动夹取闭环。
   - 当前 1 秒反馈窗口没有新增真实运动资格验收；内部标定残差不能充当末端精度。
3. **现场入口**
   - 工程：`/home/kuang/workspace/chess_robot`；Jetson AGX Orin / Ubuntu 22.04 / aarch64。
   - [图像预览](http://192.168.30.26:8088)、[夹爪点动](http://192.168.30.26:8089)；服务不自启。
   - 本次文档核对日期：**2026-10-31（Asia/Taipei）**；现场未提交改动不等于 GitHub 已更新。

## 2 从这里查阅

1. **日常使用**
   - [机械臂启动、存档、断电恢复与故障定位](docs/operations/arm-quickstart.md)。
   - [双网页、夹爪点动与服务恢复](docs/operations/web-panels.md)。
2. **开发与接口**
   - [完整文档索引：架构、包说明、构建、配置与模型](docs/README.md)。
   - [机械臂接口与执行约束](src/chess_robot_arm/README.md)、[手眼采样与求解](src/chess_robot_handeye/README.md)。
3. **结果与证据**
   - [验证索引与历史适用范围](docs/validation/README.md)。
   - [手眼标定及激光粗检](docs/validation/handeye-20261030.md)、[1 秒反馈窗口](docs/validation/feedback-window-1s-20261031.md)。

## 3 资料维护

1. **权威内容各归一处**
   - 包 README 维护构建与接口；`docs/operations/` 维护现场操作；`docs/validation/` 保存结论和证据索引。
   - `runs/` 为被 Git 忽略的现场证据、部署与可恢复备份，不随克隆分发。
   - `reference/` 与 `docs/vendor/` 保留原始资产、来源、校验值、许可证和厂家协议。
2. **模型与实现**
   - Xacro 是模型源，URDF 是生成物；Python 用于离线分析、测试与证据整理，ROS 功能节点使用 C++。
   - 历史验证只证明报告中的版本和范围；新版本按当前配置重新核对，不改写旧结果。
