# 棋子感知与场景状态

`xiangqi_perception` 消费相机图像、深度及标定信息，发布相机坐标系测量。
`xiangqi_scene` 消费这些测量及机械臂反馈，计算基座坐标、维护跟踪和稳定观察结果。
设备原始测量与识别结果分别发布；两个节点不发送机械臂动作。

从项目根目录构建：`bash tools/build.sh --packages-up-to chess_robot_perception`，然后 `source tools/env.sh`。
启动入口在 `scripts/start_perception.sh`、`start_scene.sh`；`start_live_tracked.sh` 同时管理两进程。
它们会启动节点，使用前核对当前是否已有同名实例；开发构建本身不会启动它们。

Python 推理使用已有外部 `python-yolo` 运行器，`XIANGQI_PYTHON` 可指定同依赖的解释器。
模型路径通过既有 `XIANGQI_MODEL_ARTIFACT` 等环境变量选择；源权重仍来自原训练工程。
`CHESS_ROBOT_ROOT` 选择工作区；默认标定为 `data/handeye/flange_T_camera.yaml`。
`XIANGQI_RUN_DIR` 指定本轮测量资料；脚本默认创建带时间的 runs 目录。

接口与数据含义见 [ROS 接口](../../docs/ros_interfaces.md)，稳定观察见 [观察规则](../../docs/vision/stable-observation.md)。
这是工作区源码的开发入口；现场服务仍使用既有冻结版本，尚未切换到本包。

运行日志使用 ROS logger，感知和场景沿用 3 秒标量统计；测量编码/解析异常记录首次、3 秒持续统计和恢复。完整测量保留在 runs，详见 [日志说明](../../docs/operations/logging.md)。
