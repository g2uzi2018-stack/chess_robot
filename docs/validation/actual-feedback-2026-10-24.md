# C++ 连接与真实反馈验证

日期：2026-10-24（Asia/Taipei）。需求来源：[中国象棋夹取](https://app.notion.com/p/3ff9f9a4e09e81fca280e1f3e4c42140)。

本轮已使用工程现有 SDK 参考和协议资料，完成真实控制器只读查询、反馈采集及 ROS 2 关节状态发布验证。现有资料足以开展这些工作；无需等待用户重新确认版本才能继续编码。

## 实际结果

| 验证项 | 实测结果 |
| --- | --- |
| 29999 查询 | RobotMode、GetAngle、GetPose、GetErrorID 均返回错误码 0，命令回显匹配 |
| RobotMode | 返回原始状态值 4；反馈采集中所有帧也为 4 |
| GetErrorID | 返回 `0,{[]},GetErrorID();` |
| 30004 反馈 | 约 10 秒，1225 帧，1764000 字节；长度 1440、标记、有限数值、递增时间戳检查通过 |
| 接收间隔 | 中位数 8.159364 ms，P95 8.433426 ms，最大 36.698052 ms |
| 二进制与 CSV | 对全部帧逐项核对已解码字段，一致 |
| 关节角对照 | 首帧/末帧对采集前/后 GetAngle 应答，最大绝对差小于 0.0001°；属异步静态快照对照 |
| TCP 对照 | 末帧对采集后 GetPose，应答与反馈六个数值吻合；残差记录在 verification.json |
| ROS 2 真实反馈 | 5 秒记录 622 帧；订阅到 joint1..joint6 的位置/速度，effort 为空 |
| 单位换算 | 订阅到的一条 JointState 对照同次采集 CSV 的真实帧，12 个位置/速度换算数值最大差为 0 |
| 离线回归 | 4 个测试，0 errors / failures / skipped |
| 证据防覆盖 | 用相同 prefix 重试采集被拒绝；已有 bin/csv SHA256 保持一致 |

控制器时间戳保留原始值。本次最大接收间隔体现接收链路存在波动，不能把接收间隔直接当作严格采样周期，也未据此声称零丢帧或时钟同步。静态数值吻合验证了当前读取字段与应答的对应关系，不是物理 TCP 精度、全协议兼容性或运动验收。

## 本轮代码

工程：AGX `/home/kuang/workspace/chess_robot`，分支 `feat/ag95-description-20261023`；基线 HEAD `4dbd56ad5836216e496c6bb3b78f057fa4e2e46f`。本轮与此前通信基础修改均保留为未提交工作树修改，尚未推送。

- `src/chess_robot_validation/src/dashboard_reply.hpp`：应答错误码、嵌套 payload 与命令回显解析。
- `src/chess_robot_validation/src/readonly.cpp`：新增 common 查询配置；未知配置在建连前拒绝；非零控制器错误返回退出码 3，输入/传输/解析失败为 2。
- `src/chess_robot_validation/src/feedback_frame.hpp`：从已有反馈节点提取共同帧验证；补充 speed_scaling 有限值、字段边界与时间戳序列检查。
- `src/chess_robot_validation/src/feedback_capture.cpp`：新增 C++ 采集工具；记录 bin/csv/summary，先保存收到的字节再校验；保留错误和部分帧；文件以排他创建方式打开；不发布 ROS、不发送应用数据。
- `src/chess_robot_validation/src/net.hpp`：增加有期限的分片接收，供采集器即时保留原始字节；原完整读取复用相同逻辑。
- `src/chess_robot_validation/src/feedback.cpp`：复用共享帧验证，保留原 ROS 2 发布入口。
- CMake / README：安装新程序，登记 4 个测试，补充运行方式和结果含义。

```bash
cd /home/kuang/workspace/chess_robot
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select chess_robot_validation --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --base-paths src --packages-select chess_robot_validation --event-handlers console_direct+
colcon test-result --test-result-base build/chess_robot_validation --verbose

ros2 run chess_robot_validation nova5_readonly --query 192.168.30.234 common 'GetAngle()'
# 创建新的、未使用过的证据目录，再运行以下采集命令：
ros2 run chess_robot_validation nova5_feedback_capture 192.168.30.234 10 runs/<new-run-id>/actual
```

ROS 2 验证使用 `ROS_DOMAIN_ID=62 ROS_LOCALHOST_ONLY=1`。现有发布程序的 v4 参数用于选择本项目协议参考，共同字段已由本次实测交叉核对；并未从该参数反推设备固件版本。

## 证据与资料

- `runs/20261024_readonly_actual_US4egh/`：实际查询原始应答、10 秒 bin/csv/summary、5 秒 ROS 采集、订阅记录、逐字段 verification.json、防覆盖 SHA256 与拒绝日志。
- `runs/20261024_feedback_protocol/`：构建与测试日志、离线核对脚本、操作脚本、协议文本查阅中间件。
- SDK 字段依据：`reference/dobot/communication/include/dobot_bringup/command.h`，来源为原厂 [DOBOT_6Axis_ROS2_V4](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4/tree/f31d2ed49c9ed74ac43dc355da74f6b1b6911f87)。
- 协议依据：工程已保存的 V4.6.6 文档；来源为原厂 [TCP-IP-Python-V4 固定提交](https://github.com/Dobot-Arm/TCP-IP-Python-V4/tree/55ec1ec82201aaf2e6d54aa47b6272bbaa14c815)。另只读查阅原厂 [V3 查询实现与共同反馈字段](https://github.com/Dobot-Arm/TCP-IP-Python-V3/blob/main/dobot_api.py)。没有运行厂家演示或旧业务程序。

下一步可在现有只读链路上继续实现厂家 FK/IK 与模型坐标对照。安装/TCP 标定和完整碰撞场景仍是之后实机夹取需要解决的工程项；不影响本轮已完成的通信与真实反馈验证。
