cmake_minimum_required(VERSION 3.16)
project(chess_robot_interface)

# 构建工具与接口代码生成。
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 接口字段引用的标准类型。
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# 集中生成项目使用的消息、服务和动作。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmState.msg"
  "msg/FrameInfo.msg"
  "msg/GripperState.msg"
  "srv/ConfirmGrasp.srv"
  "srv/InitializeReference.srv"
  "srv/RestoreCheckpoint.srv"
  "srv/RestoreReference.srv"
  "action/MoveArm.action"
  "action/MoveGripper.action"
  "action/MoveJoints.action"
  DEPENDENCIES geometry_msgs std_msgs trajectory_msgs
)

install(FILES README.txt DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
