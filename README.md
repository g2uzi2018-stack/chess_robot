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
  chess_robot_vision/        # D405 取流、校正、圆候选与表面三维点
  chess_robot_preview/       # 按需订阅、30 Hz 局域网预览面板
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
- 已实现机械臂控制测试包、D405 视觉基线与独立预览节点；尚未形成自动夹取闭环。
- 已整合 AG95 和用户更新的相机/支架，按新 CAD 配准安装基准，提供独立夹爪和组合 URDF；实机 TCP 等仍待标定。
- [导出模型供在线解析器查看](docs/models/ag95-browser.md)：提供纯 URDF 与相对路径网格，不附带本地查看器。
- 现有 [nova](https://github.com/g2uzi2018-stack/nova) 项目中有已能控制机械臂的参考代码（用户确认）；旧业务代码未迁入。
- 控制器固件与末端工具配置仍需复核，相机外参和棋盘参数待标定；当前模型的原厂速度/力矩占位值尚未替换为实机配置。

## 视觉与预览

- D405 直连 AGX，默认 640×480、30 Hz；视觉节点输出未分类圆候选及相机光学坐标系中的表面点，分类与手眼标定仍待验收。
- [视觉包](src/chess_robot_vision/README.md) 与 [预览包](src/chess_robot_preview/README.md) 包含构建、启动和接口说明。
- 当前 AGX 面板：<http://192.168.30.26:8088>；未点击的图像不加载，当前组合服务为 `chess_robot_preview.service`，ROS domain 90，未启用开机自启。
- [30 Hz 实测报告](docs/validation/preview-30hz-20261025.md)：局域网不同帧实收约 29.4～29.7 fps；[机器可读证据](docs/validation/vision-preview-20261025/README.md)。

## 资料入口

- [模型包](src/chess_robot_description/README.md)
- [AG95 夹爪模型](src/dh_ag95_description/README.md)
- [AI 辅助 URDF 装配验证路线与工具](docs/models/urdf-workflow.md)
- [给后续 AI / 工程师的接力说明](docs/models/urdf-handoff.md)
- [协议资料](docs/vendor/dobot/README.md)
- [参考材料与来源](reference/dobot/README.md)

初始化日期：2026-10-20（Asia/Taipei）。
