AG95 夹爪模型入口

文件用途
  dh_ag95_macro.xacro：可复用的夹爪定义，包含部件、连接、联动和显示/碰撞形状。
  ag95.urdf.xacro：单独使用夹爪模型的可编辑入口，指定模型参数并调用夹爪定义。
  ag95.urdf：由独立入口生成的完整夹爪描述，不手工修改。

使用关系
  ../../README.md：参数含义、模型边界和来源说明。
  ../../SOURCES.json：来源仓库、版本、原件文件指纹和本地调整说明。
  整机模型在 ../nova5_with_ag95.urdf.xacro 中引用夹爪定义。
  夹爪本体改动写在本目录；安装到机械臂的位置写在整机模型入口。
  单独查看时加载 ag95.urdf，并提供模型包的 meshes/ag95/ 资源。

生成命令
  从项目根目录执行，前提是 ROS 2 Humble 环境中已安装 Xacro：
    source /opt/ros/humble/setup.bash
    python3 tools/description/generate_models.py
  现有脚本一次更新 AG95 独立模型和整机模型两份 URDF。
  仅在输入变化影响生成结果、且需要使用或维护该结果时执行；
  本次需要的结果缺失或已确认过期时，在使用前补生成。
  阅读、文档、说明性注释或无关代码调整，不运行生成或 --check。
  仅换同路径网格且 URDF 参数未变时，不因此重写 URDF。
  生成和验证分别判断，不默认运行整套检查。

详细说明
  工程查看用法：../../../../docs/models/ag95-browser.md。
  组装与验证工作流：https://app.notion.com/p/4039f9a4e09e81d98ba4eb3f8b8597cc
