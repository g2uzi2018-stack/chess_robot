机械臂 C++ 接口提示

这里是可直接调用的核心库接口；只通过 ROS 通信的节点无需依赖这些头文件。
使用时包含 chess_robot_arm/<文件名>，CMake 查找 chess_robot_arm 并链接导出的
chess_robot_arm::chess_robot_arm_core 目标。ROS 接口定义在 chess_robot_interface，说明见 docs/ros_interfaces.md。

- core.hpp：Config、Goal、Plan；State 保存实测状态/线缆历史，TaskGate 管理任务，
  Planner 规划，Executor 执行；Kinematics/Driver 声明运动学和设备操作接口。
- driver.hpp：V4Driver 实现真实控制器通信；提供反馈解码和 SDK 单位/姿态转换。
- geometry_guard.hpp：GeometryGuard 检查模型距离与姿态条件；validateTableContactGoal 检查接触任务。
- checkpoint.hpp：CheckpointStore 保存、读取完整历史存档，运动前标记旧档不可恢复。
- dashboard_reply.hpp：parse_dashboard_reply 解析命令回复并核对回显；维护工具直接复用。
- feedback_frame.hpp：FeedbackFrame、FeedbackSequence，定义原始反馈布局并识别重复/回退时间戳。
- net.hpp：Socket 和 QueryFailure，处理带期限的 TCP 收发并保留不完整回包。
- rotating_audit.hpp：AuditDirectory 管理写入归属，RotatingAudit 异步记录并轮换文件。

现有外部使用者包括 tools/arm/src/operator.cpp 和 tools/analysis/workspace_policy_probe.cpp。
本库内部采用 m/rad；厂家 mm/deg 换算在 driver 层完成。
