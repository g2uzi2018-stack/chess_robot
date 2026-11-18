chess_robot_interface 接口索引

本包集中维护相机、机械臂和夹爪的自定义通信接口，供相关 ROS 2 包依赖使用，无需单独启动。

消息 msg/
  FrameInfo.msg：同一组彩深图的帧序号、设备时间、深度比例和流状态。
  ArmState.msg：机械臂的运行阶段、末端位姿、参考状态和绕转信息。
  GripperState.msg：夹爪的连接、初始化、开度和夹持状态。

服务 srv/
  ConfirmGrasp.srv：提交指定任务的夹持确认结果。
  InitializeReference.srv：初始化机械臂的软件参考，不修改编码器零点。
  RestoreReference.srv：依据审计记录恢复机械臂参考状态。
  RestoreCheckpoint.srv：从可信停稳存档恢复机械臂参考状态。

动作 action/
  MoveArm.action：机械臂位姿运动、回 Home 及抓取流程的规划或执行。
  MoveJoints.action：机械臂手动关节目标的规划或执行。
  MoveGripper.action：夹爪初始化、开合及夹持操作。

详细说明
  字段结构及含义：查看对应的 .msg、.srv、.action 文件。
  接口名称、提供方、使用方和交互约定：查看项目根目录 docs/ros_interfaces.md。
  从本目录访问该文档的相对路径：../../docs/ros_interfaces.md。

接口新增、删除或用途变化时，同步更新本索引。
