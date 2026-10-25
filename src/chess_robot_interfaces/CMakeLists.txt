cmake_minimum_required(VERSION 3.16)
project(chess_robot_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveArm.action" "msg/ArmState.msg"
  "srv/ConfirmGrasp.srv" "srv/InitializeReference.srv" "srv/RestoreReference.srv"
  DEPENDENCIES geometry_msgs std_msgs trajectory_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
