cmake_minimum_required(VERSION 3.16)
project(chess_robot_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 构建工具。
find_package(ament_cmake REQUIRED)

# 节点通信、图像消息和相机 SDK。
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(chess_robot_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(realsense2 REQUIRED)

# 包内校正库，供节点和已有离线检查复用，不作为公共库安装或导出。
add_library(camera_calibration STATIC src/calibration.cpp)
target_include_directories(camera_calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(camera_calibration PUBLIC
  ${OpenCV_LIBS}
  realsense2::realsense2
)
target_compile_options(camera_calibration PRIVATE -Wall -Wextra -Wpedantic)

# D405 采集、校正及 ROS 消息发布。
add_executable(vision_node src/node.cpp)
ament_target_dependencies(vision_node
  rclcpp
  rcl_interfaces
  chess_robot_interface
  std_msgs
  sensor_msgs
  cv_bridge
)
target_link_libraries(vision_node
  camera_calibration
  realsense2::realsense2
)
target_compile_options(vision_node PRIVATE -Wall -Wextra -Wpedantic)

# 保留外部 SDK 库的运行时搜索路径，支持 SDK 安装在自定义前缀。
set_target_properties(vision_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

# 安装节点及运行资源；config 中的系统配置仍需按文件说明另行部署。
install(TARGETS vision_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

# 既有校正测试，仅在启用 BUILD_TESTING 时参与构建。
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_camera_calibration test/test_calibration.cpp)
  target_link_libraries(test_camera_calibration camera_calibration)
endif()

ament_package()
