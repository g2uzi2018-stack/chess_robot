Nova5 + AG95 装配入口

文件用途
  ag95/：夹爪本体与独立模型入口，见 ag95/README.txt。
  nova5.urdf：Nova5 原厂本体描述，是总装使用的输入，保留原厂基线。
  nova5_with_ag95.urdf.xacro：可编辑的总装入口，组合机械臂、安装件和 AG95 夹爪。
  nova5_with_ag95.urdf：由总装入口生成，供现有工具和节点读取，不手工修改。

修改位置
  整机安装关系与可调参数：nova5_with_ag95.urdf.xacro。
  夹爪本体定义：ag95/dh_ag95_macro.xacro。
  安装件网格及来源：../meshes/tool_mount/README.txt。
  参数含义、默认值和已知限制：../README.md。

生成命令
  从项目根目录执行，前提是 ROS 2 Humble 环境中已安装 Xacro：
    source /opt/ros/humble/setup.bash
    python3 tools/description/generate_models.py
  现有脚本一次更新 AG95 独立模型和整机模型两份 URDF，执行前确认这个范围。
  仅在输入变化影响生成结果、且需要使用或维护该结果时执行；
  本次需要的结果缺失或已确认过期时，在使用前补生成。
  阅读、文档、说明性注释或无关代码调整，不运行生成或 --check。
  仅换同路径网格且 URDF 参数未变时，不因此重写 URDF。
  生成完成不自动触发全部模型检查或实机验证。

查看与加载
  使用生成的 nova5_with_ag95.urdf；查看器须能解析 chess_robot_description 的 package:// 路径。
  导出可搬运的查看文件：项目根目录运行 python3 tools/description/export_browser_model.py。
  工程用法：../../../docs/models/ag95-browser.md。
  详细工作流：https://app.notion.com/p/4039f9a4e09e81d98ba4eb3f8b8597cc
