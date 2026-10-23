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
tools/description/          # 模型生成、检查和浏览器导出工具
```

## 当前状态

- 开发目录：`~/workspace/chess_robot`。
- 开发机基线：Jetson / aarch64 / Ubuntu 22.04.5 LTS。
- 已迁入原厂 Nova5 本体模型和协议资料，功能代码尚未开始。
- 已迁入旧 `nova` 工程的 AG95 夹爪和支架，保留原安装关系，提供独立夹爪和组合纯 URDF。
- [在 AGX 浏览器中查看组合模型](docs/models/ag95-browser.md)：无需连接机械臂或安装网页服务。
- 现有 [nova](https://github.com/g2uzi2018-stack/nova) 项目中有已能控制机械臂的参考代码（用户确认）；旧业务代码未迁入。
- 控制器固件、末端工具、相机和棋盘参数待确认；当前模型的原厂速度/力矩占位值尚未替换为实机配置。

## 资料入口

- [模型包](src/chess_robot_description/README.md)
- [AG95 夹爪模型](src/dh_ag95_description/README.md)
- [协议资料](docs/vendor/dobot/README.md)
- [参考材料与来源](reference/dobot/README.md)

初始化日期：2026-10-20（Asia/Taipei）。
