cmake_minimum_required(VERSION 3.8)
project(chess_robot_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# Python 源码放在 python/ 下，安装后仍用原包名导入。
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR python/${PROJECT_NAME}
)

install(DIRECTORY urdf meshes DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE LICENSE.ag95 SOURCES.json DESTINATION share/${PROJECT_NAME})

ament_package()
