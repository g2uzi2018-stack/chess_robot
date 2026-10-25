# AG95 夹爪节点首版验证（2026-10-25）

## 1 结果

新增 ROS 2 Humble / C++17 包 `chess_robot_gripper`、`gripper_node`，以及共享接口
`MoveGripper.action` / `GripperState.msg`。提供初始化、原始开度/力值命令、反馈判定、
忙闲互斥和模拟后端；硬件使用 Nova5 末端 RS485 的 RTU 透传端口。

这是**软件首版与真实只读通信验证**。实物初始化、开合、棋子夹持和掉落实验尚未进行，
不能认定自动夹取闭环已验收。

开发基线：`e5226f35b4c86d7b03763cb5124d264b675725ca`。
独立分支：`feat/gripper-node-20261025`。
AGX checkout：`/home/kuang/workspace/chess_robot_gripper`。
原始机械臂测试工作区 `/home/kuang/workspace/chess_robot` 的源文件、构建和运行程序未被修改。

## 2 接口

- `/gripper/command`：`chess_robot_interfaces/action/MoveGripper`。
  `operation=0` 单向初始化、`1` 设置位置；`position_raw=0..1000`，
  `force_percent=20..max_force_percent`（默认上限30），`require_contact` 指定是否必须夹住物体，
  `timeout_sec=0.5..60` 必填。
- Action 结果0初始化、1到位、2夹住；10未初始化、11空夹、12非预期接触、13掉落、
  14超时、15通信错误、16关闭。`command_attempted`包含已发送但回包丢失的不确定情况。
- `/gripper/state`：10 Hz，包含模拟标志、连接、反馈有效性/年龄、忙闲、命令启用、
  初始化、夹持状态、原始开度、物体与掉落标志。
- 未标定毫米开口、关节角和实际夹持力，不发布伪造的 JointState。

## 3 验证证据

- 在 AGX Orin / Ubuntu22.04 / ROS2 Humble 编译共享接口和节点。
- 18项 C++ 单元测试覆盖厂家初始化样例CRC、写回显、读值、错误CRC/从站/功能码、
  异常回包、截断/长度、无写重试、参数范围、两个新样本确认、旧held状态、空夹、
  接触、掉落、初始化丢失及模拟驱动。
- 完整节点 ROS Action / TCP 回环使用 `ROS_DOMAIN_ID=87`、`ROS_LOCALHOST_ONLY=1`；
  协议模拟器仅监听 `127.0.0.1`。覆盖未初始化拒绝、初始化与已初始化无写确认、
  非法目标、忙时拒绝、取消明确拒绝、硬件只读零写、拆包、空夹显式恢复、接触/掉落、
  坏回包、超时、旧夹持状态、反馈失效、只读重连和不自动重发命令。
- 真实 C++ 节点使用 `ROS_DOMAIN_ID=88` 连 `192.168.30.234:60000`、从站1，
  `enable_commands=false`。采集10条有效非模拟反馈；初始化0、夹持状态1、开度1000。
  未发送Action目标，未发送初始化、位置/力值、工具模式/供电或机械臂运动指令。

同目录 `unit-results.xml`、`integration-result.json`、`hardware-readonly.json`、
`build.log`、`integration.log` 为本轮证据。可复现命令见包README和测试脚本。

## 4 边界与后续

- 默认 launch 是可接收命令的 mock；直接运行可执行文件默认 mock 且禁用命令。
  `config/hardware.yaml` 为现场端口的只读配置，真实动作须在现场配置显式启用。
- 不更改 Nova5 共享RS485格式或电源，不通过Dashboard创建/释放Modbus主站。
- CRC正确的写回显不算到位；连续新鲜反馈确认。通信故障时未知哨兵值替代旧数据，
  object_detected/object_dropped置false。
- 不自动初始化、全行程校准、保存Flash或重发失败写命令。
- 手册未列独立停止寄存器，Action取消明确拒绝。客户端断线、超时、节点退出不能保证
  物理停止，不会自行张开或切断工具电源。超时/通信不确定时锁定新命令并继续监测；
  需检查实物后重启。空夹/接触/掉落为可处理业务结果，不自动重放。
- 实物开合需与正在测试的机械臂统一调度，不能让两个独立程序同时改变同一套几何。
  后续应验证开合方向、初始回零、安装限位、棋子所需力值、真实夹持/掉落反馈，
  并把有效夹爪几何和夹持状态接入机械臂规划/撤离互锁。

需求来源：[中国象棋夹取](https://app.notion.com/p/3ff9f9a4e09e81fca280e1f3e4c42140)。
寄存器依据：[大寰AG系列操作手册V2.6](https://www.dh-robotics.com/wp-content/uploads/2022/12/AG%E7%B3%BB%E5%88%97-%E6%93%8D%E4%BD%9C%E8%AF%B4%E6%98%8E%E4%B9%A6MODBUS-%E7%89%88V2.6-%E5%A4%A7%E5%AF%B0%E6%9C%BA%E5%99%A8%E4%BA%BA.pdf)。
