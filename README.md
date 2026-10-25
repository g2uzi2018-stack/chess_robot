# 中国象棋夹取

使用越疆 Nova5 机械臂开展中国象棋棋子的识别、定位与夹取。

## 开发方向

- ROS 2 集成与功能代码使用 C++ 从头实现。
- 机械臂基础运动与正逆运动学，优先使用与设备匹配的原厂控制器接口。
- 保留原厂模型和协议资料；通信代码与示例仅作为参考材料。

## 目录

```text
src/
  chess_robot_description/   # Nova5 本体、支架与 AG95 组合模型
  dh_ag95_description/       # AG95 夹爪 STL、关节与独立预览模型
docs/
  vendor/dobot/             # 原厂 TCP/IP 协议与版本说明
reference/
  dobot/                    # 原厂参考源码、示例、来源与校验值
  ag95/                     # 旧装配对照件与迁移来源记录
tools/description/          # 模型生成、检查和纯 URDF 资源导出工具
```

## 当前状态

- 开发目录：`~/workspace/chess_robot`。
- 开发机基线：Jetson / aarch64 / Ubuntu 22.04.5 LTS。
- 已迁入原厂 Nova5 本体模型和协议资料，功能代码尚未开始。
- 已整合 AG95 和用户更新的相机/支架，按新 CAD 配准安装基准，提供独立夹爪和组合 URDF；实机 TCP 等仍待标定。
- [导出模型供在线解析器查看](docs/models/ag95-browser.md)：提供纯 URDF 与相对路径网格，不附带本地查看器。
- 现有 [nova](https://github.com/g2uzi2018-stack/nova) 项目中有已能控制机械臂的参考代码（用户确认）；旧业务代码未迁入。
- 控制器固件、末端工具、相机和棋盘参数待确认；当前模型的原厂速度/力矩占位值尚未替换为实机配置。

## 资料入口

- [模型包](src/chess_robot_description/README.md)
- [AG95 夹爪模型](src/dh_ag95_description/README.md)
- [AI 辅助 URDF 装配验证路线与工具](docs/models/urdf-workflow.md)
- [给后续 AI / 工程师的接力说明](docs/models/urdf-handoff.md)
- [协议资料](docs/vendor/dobot/README.md)
- [参考材料与来源](reference/dobot/README.md)

初始化日期：2026-10-20（Asia/Taipei）。

## 夹爪控制（测试中）

- `src/chess_robot_gripper/`：独立 C++ AG95 命令与反馈节点，支持模拟和 Nova5 末端 RS485 透传。
- 命令 `/gripper/command`、状态 `/gripper/state`；启动和接口见[夹爪节点说明](src/chess_robot_gripper/README.md)。
- 已完成软件回归和真实只读反馈验证；实物开合/棋子夹持仍待验收。
- 证据见[首版验证报告](docs/validation/gripper-node-20261025/REPORT.md)。
