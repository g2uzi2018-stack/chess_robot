# 原厂参考材料

本目录仅供阅读，不加入 colcon 或业务程序的构建。

## 内容

- `communication/`：原厂 Socket 和控制/反馈报文处理代码，共 4 个文件。
- `examples/demo.py`：原厂命令调用示例；原始示例使用 Python，仅参考接口用法。新项目功能代码使用 C++。
- `models/nova5_robot.urdf`：未经修改的原厂模型对照件。
- `VENDOR_SOURCES.json`：来源提交、路径、修改说明与 SHA-256。
- `SHA256SUMS`：本次迁入的原厂资产和文档的校验值。

## 来源与版本

- 来源仓库：https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4
- 代码与模型基线：`f31d2ed49c9ed74ac43dc355da74f6b1b6911f87`，与原 Nova 归档记录一致。
- 从 Nova 归档提交 `a45852ab1d414a1fe075a98f47551ca846a783f6` 提取上述原厂文件，没有复制其业务改动。
- `LICENSE` 保留官方仓库提交 `ec201c40d5db63185a3593ec9c39f80fe25aa527` 中的 MIT 许可证；该文件来自当前官方版本，与归档代码基线分别记录。

## 阅读说明

原厂通信代码仍引用 rclcpp 等头文件，不是本项目已集成的独立 C++ SDK；不要直接作为已验证的新驱动编译。
示例含机器人使能和运动操作，本次未执行。后续需要实现的接口应按协议和实机行为逐项封装。
