# 中国象棋夹取

使用越疆 Nova5 机械臂开展中国象棋棋子的识别、定位与夹取。

> 本工作区为 `feat/gripper-node-20261025`，基于215d07b。夹爪面板、初始化修复和本次文档属于尚未提交的现场改动，尚未合入main。
> 视觉、预览和手眼标定的当前代码及运行实例位于主工作区 `/home/kuang/workspace/chess_robot`，不要将本分支旧基线视为全项目最新状态。

## 开发方向

- ROS 2 集成与功能代码使用 C++ 从头实现。
- 机械臂基础运动与正逆运动学，优先使用与设备匹配的原厂控制器接口。
- 保留原厂模型和协议资料；通信代码与示例仅作为参考材料。

## 目录

```text
src/
  chess_robot_description/   # Nova5 本体、支架与 AG95 组合模型
  dh_ag95_description/       # AG95 夹爪 STL、关节与独立预览模型
  chess_robot_arm/           # 机械臂控制（本分支基线版本）
  chess_robot_gripper/       # AG95 独立Action命令与真实反馈
  chess_robot_panel/         # 浏览器手动点动面板（本地新增）
docs/
  vendor/dobot/             # 原厂 TCP/IP 协议与版本说明
reference/
  dobot/                    # 原厂参考源码、示例、来源与校验值
  ag95/                     # 旧装配对照件与迁移来源记录
tools/description/          # 模型生成、检查和纯 URDF 资源导出工具
```

## 当前状态

- 本工作区：`~/workspace/chess_robot_gripper`；主工作区：`~/workspace/chess_robot`。
- 开发机基线：Jetson / aarch64 / Ubuntu 22.04.5 LTS。
- 已具备机械臂受限运动、夹爪控制与网页点动能力；主工作区另有相机、预览和手眼标定，自动夹取闭环尚未完成。
- 已整合 AG95 和用户更新的相机/支架，按新 CAD 配准安装基准，提供独立夹爪和组合 URDF；实机 TCP 等仍待标定。
- [导出模型供在线解析器查看](docs/models/ag95-browser.md)：提供纯 URDF 与相对路径网格，不附带本地查看器。
- 现有 [nova](https://github.com/g2uzi2018-stack/nova) 项目中有已能控制机械臂的参考代码（用户确认）；旧业务代码未迁入。
- 现场为Nova5＋AG95＋D405 / ROS 2 Humble；真实夹指TCP、毫米开口、实际夹持力和自动夹取精度仍待验收。当前模型的原厂速度/力矩占位值尚未替换为实机配置。

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
- 已有20%力设定下1000→0实机逐步收窄记录，现已部署手动网页并收到用户操作的开合反馈；棋子夹持、掉落和量化精度仍未验收。
- 证据见[首版验证报告](docs/validation/gripper-node-20261025/REPORT.md)。
- 当前面板、点动和初始化修复见[2026-10-31记录](docs/validation/manual-panel-20261031.md)。

## 局域网操作入口

- [图像预览](http://192.168.30.26:8088) 与 [夹爪控制](http://192.168.30.26:8089) 页头可互相跳转。
- 控制页短按一步、按住连续、松开停止追加；默认20 raw、20%力，支持1..100 raw步长，指定开度在折叠区。
- 操作、部署与恢复见[双网页操作说明](docs/operations/web-panels.md)及[面板README](src/chess_robot_panel/README.md)。
