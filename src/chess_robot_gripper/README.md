# AG95 独立夹爪节点

ROS 2 Humble / C++17。`gripper_node` 只控制夹爪，不使能、停止或移动机械臂。

## 连接与当前验证

现场 AG95 接在 Nova5 **末端 RS485**。2026-10-25 从 AGX 经
`192.168.30.234:60000` 直接收发 Modbus **RTU 二进制帧**，已只读确认
`0x0200=0`（未初始化）、`0x0201=1`（到位）、`0x0202=1000`（开度原始值）。
这不是带 MBAP 头的 Modbus TCP。无需经过 Dashboard 的 `ModbusRTUCreate`。
节点不自动更改 `SetTool485`、工具电源或工作模式，也不创建/释放其他程序的 Modbus 主站。
默认从站为 1；串口格式取决于控制器已有配置，本次只读成功未重新配置它。

已完成软件回归、真实只读反馈，以及20%力设定下1000→0、步长100的11点实机收窄验证。
一次初始化请求后出现范围异常并中止；只读确认已初始化后，第二批未重发初始化、11点全部反馈到位。
见[实机收窄报告](../../docs/validation/gripper-sweep-20261025/REPORT.md)。毫米开口、实际力和棋子夹持/掉落仍未验收。
并行机械臂测试须核对实际使用的碰撞包络；本次已确认当前配置合并21种模型开度。
独立ROS节点本身不构成机械安全隔离。

## 启动

在包含本包和新接口的工作区：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to chess_robot_gripper
source install/setup.bash
# 默认 launch 使用 mock.yaml，只产生模拟反馈，允许模拟命令。
ros2 launch chess_robot_gripper gripper.launch.py
# 真实设备只读（hardware.yaml 的 enable_commands=false）。
ros2 launch chess_robot_gripper gripper.launch.py \
  config:=$(ros2 pkg prefix --share chess_robot_gripper)/config/hardware.yaml
```

直接 `ros2 run chess_robot_gripper gripper_node` 默认 mock 且禁止命令。
需实际控制时，复制 hardware.yaml 为现场配置、确认机械臂静止及夹爪空间，再显式设置
`enable_commands: true`。所有参数均为启动参数，运行时修改会被拒绝。

## 接口与单位

`/gripper/command`：`chess_robot_interfaces/action/MoveGripper`。

| 目标字段 | 含义 |
|---|---|
| `operation` | `0 INITIALIZE` 单向初始化；`1 MOVE` 设置原始开度 |
| `position_raw` | MOVE 的开度 0..1000；旧工程实测 0 闭合、1000 全开，当前安装映射还需复核 |
| `force_percent` | MOVE 的力设定百分比，厂家范围 20..100，默认现场上限 30；不是牛顿或实测力 |
| `require_contact` | true 必须收到新的“夹住物体”反馈；空夹到位返回失败 |
| `timeout_sec` | 0.5..60 秒，必须显式填写，包含命令收发和后续等待 |

不提供未经标定的毫米开口、URDF 关节角、牛顿力值或 JointState。原装指垫的非线性
运动关系、实际夹持位置和几何映射要另行标定。

```bash
# 初始化有物理动作；仅在明确可执行时发送。
ros2 action send_goal /gripper/command chess_robot_interfaces/action/MoveGripper \
  '{operation: 0, timeout_sec: 5.0}' --feedback
# 模拟张开：硬件使用前必须复核开合方向和现场条件。
ros2 action send_goal /gripper/command chess_robot_interfaces/action/MoveGripper \
  '{operation: 1, position_raw: 1000, force_percent: 20, require_contact: false, timeout_sec: 5.0}' --feedback
# 模拟夹持：用 mock_contact_raw:=450 启动可模拟在450处接触。
ros2 action send_goal /gripper/command chess_robot_interfaces/action/MoveGripper \
  '{operation: 1, position_raw: 0, force_percent: 20, require_contact: true, timeout_sec: 5.0}' --feedback
```

返回 `code`：0 初始化已确认、1 到位、2 夹持；10 未初始化、11 空夹、12 非预期接触、
13 掉落、14 超时、15 通信错误、16 关闭。0/1/2 为成功，其余为 Action aborted。
已初始化时的 INITIALIZE 只重新读取确认，不重复回零。
`command_attempted` 表示可能已发出写命令；丢回包时也为 true。
`simulated` 必须由调用方核对，模拟成功不代表硬件验收。

`/gripper/state`：`GripperState`，10 Hz。包含连接、反馈有效性、反馈年龄（秒）、忙闲、
命令启用标志、初始化、夹持状态、原始开度、物体/掉落标志及说明。
`header.stamp` 是发布时刻；样本年龄使用单调时钟，从三条顺序读寄存器的开始时刻计算，
不声称三个寄存器是原子快照。失联/过期时状态为255、开度65535，物体标志无效并置false。

## 完成条件与故障

- 单工作线程串行处理 I/O；同一时刻只接收一个 Action，忙时拒绝新目标。
- 未初始化不接收 MOVE；不在启动、重连或收取命令时自动初始化。
- 先写力0x0101，再写位置0x0103；校验CRC、从站、功能码、长度及完整写回显。
  成功回显只是命令确认，必须由连续两次新鲜反馈确认到位或夹持。
- 到位必须满足位置误差（默认5原始单位）。夹持必须有指令后的状态/位置变化；旧的held
  标志不能直接满足新的抓取任务。未观察到变化会超时，需检查而不是猜测成功。
- 不自动重放失败写命令。发命令后超时、通信失败或关闭时锁定新命令，继续只读监测。
  现场检查已停止/物体状态后重启节点；重启不会使夹爪自动运动。
  空夹、非预期接触、掉落返回业务失败；新鲜静止反馈可允许显式的新任务，例如张开恢复。
- **取消明确拒绝**：当前手册没有可独立验证的停止寄存器。Action超时、客户端断线、
  节点退出均不保证物理停止，也不会自动张开、掉电或修改机械臂状态。
- 设置力值本身也可能影响原有夹持；故障后的恢复必须确认实物状态。
- 不向Flash写入参数，不发送0xA5全行程校准。真实开合须与机械臂任务统一调度，
  实际抓取前还需把夹爪有效几何/反馈接入机械臂碰撞检查和夹持互锁。

## 资料与测试

- 需求：[中国象棋夹取](https://app.notion.com/p/3ff9f9a4e09e81fca280e1f3e4c42140)。
- [大寰AG手册V2.6](https://www.dh-robotics.com/wp-content/uploads/2022/12/AG%E7%B3%BB%E5%88%97-%E6%93%8D%E4%BD%9C%E8%AF%B4%E6%98%8E%E4%B9%A6MODBUS-%E7%89%88V2.6-%E5%A4%A7%E5%AF%B0%E6%9C%BA%E5%99%A8%E4%BA%BA.pdf)：第11–16页寄存器与反馈。
- 旧项目参考：`g2uzi2018-stack/nova` 的 `02_business_grasp/nova5_control/ag95_driver.py`，
  仅参考透传方式/开合约定，新节点从头实现校验和状态判断，不沿用任意回包即成功的行为。

```bash
colcon test --packages-select chess_robot_gripper --event-handlers console_direct+
colcon test-result --test-result-base build/chess_robot_gripper/test_results --verbose
# 独立ROS_DOMAIN_ID和localhost模式，所有网络故障测试只连127.0.0.1。
ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 python3 tools/testing/gripper_ros_integration.py
```
