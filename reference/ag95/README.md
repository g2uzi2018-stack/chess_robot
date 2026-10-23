# AG95 模型迁移来源

来源仓库：<https://github.com/g2uzi2018-stack/nova>

归档提交：`a45852ab1d414a1fe075a98f47551ca846a783f6`。
原始夹爪位于 `02_business_grasp/ros2_ws_src/dh_ag95_gripper_ros2/dh_ag95_description`，
安装组合位于 `02_business_grasp/ros2_ws_src/my_nova5_config`。

- `nova5_with_ag95.original.xacro`：未经修改的原装配对照件。
- `dh_ag95.original.urdf.xacro`：原独立入口对照件，包含旧硬件插件引用，仅供查阅。
- `package.original.xml`：原模型包元数据，声明 Apache-2.0。
- `SOURCES.json`：所有直接迁入文件的来源路径、目标路径和 SHA-256。
- 夹爪原始 Apache-2.0 许可证保存在 `src/dh_ag95_description/LICENSE`。

本次迁移保留全部 12 个夹爪 STL、原夹爪宏、原许可证、实际装配引用的一个支架 STL；
没有迁入旧驱动、启动器、控制参数或手眼标定。新建数据型模型包和无硬件预览入口。
装配链、网格 origin、夹爪安装与抓取坐标系保持旧值；地面改为可选，默认不包含。
新增纯 URDF 与浏览器导出工具。原始 Nova5 本体 URDF 和 STL 均未更改。
