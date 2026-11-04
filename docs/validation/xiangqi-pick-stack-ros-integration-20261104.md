# 象棋抓放ROS执行接口接入验证

日期：2026-11-04（Asia/Taipei）。仓库基准 `19d1281750ad517eaf48c554f4bce60d44a6c848`。

## 完成内容

`tools/pick_stack/ros_io.py`已经实现 `Demo.run(RealIO)` 所需的真实接口，不再是仅只规划的适配：现有扫描主控、机械臂分段规划/执行、夹爪MOVE、新夹持与释放反馈、持物掉落监测、人工承托/放稳确认、正式Home和失败取消均已接通。CLI `--execute`绑定真实入口，运行前需要本轮授权记录、完整量测和绑定参数/模型的审查资料。默认配置未填实物量测，保持动作前阻止。

机械臂/夹爪目标通过 `/arm/move`、`/gripper/command` Action，不直接向状态topic发命令。scene与反馈复用现有topic；新增 `/pick_stack/status`、`/pick_stack/confirmation_request`、`/pick_stack/confirmation`，确认须匹配task_id、request_id、kind、layer，并带观察人和本次实物观察。无自动确认、自动初始化、恢复参考、清错、使能或服务管理。

扫描主控只新增可选node_name及Action client退出清理，原独立扫描入口和运动规则保持原行为。仓库中其他视觉改动全部保留；本轮没有commit/push或部署。

## 隔离ROS Action契约验证

使用临时语义夹具、独立 `ROS_DOMAIN_ID=194` 和 `ROS_LOCALHOST_ONLY=1`。仅运行模拟Action server和合成反馈/场景，**没有启动硬件驱动，没有向生产domain90发送执行目标**。夹具反馈设置由场景构造以验证真实适配的字段检查，不能当作物理设备或IK成功证据。

| 场景 | 核对结果 |
|---|---|
| 完整单枚 | 现有扫描→Home→三次新窗口观察→下降→新夹持→搬运→承托→释放→撤离→堆顶观察/放稳→计1层→正式Home |
| 规划拒绝下降 | 不执行下降，不闭合抓取，不计层；保留错误，无故障后Home |
| 搬运时掉落 | 持物监控触发，取消本客户端拥有的机械臂目标；不释放、不继续搬运、不计层 |
| 释放返回失败 | 物体状态unknown，保持0层，不把开爪尝试当放置成功，不自动开爪或Home |
| 错误/旧请求编号 | 人工确认超时，未发机械臂或夹爪目标，保持0层 |

所有夹爪目标均为MOVE，没有INITIALIZE。抓取require_contact=true，开爪/释放false；新GRASPED必须有指令后的真实标志、反馈时效和新状态变化。Action结果可能早于对应topic到达的次序问题已经修正：经核验的最终样本进入单调缓存，旧topic不能倒灌成掉落。

抓取下降的末次新观测放在规划、人工确认和最后停稳/参数检查之后；不能复用规划前已经过期的时间戳。每次实体适配请求还要求已完成precheck。临时验证脚本与临时场景目录用后删除，无持久化测试文件，未使用冒烟测试。

结果见[机器可读摘要](xiangqi-pick-stack-ros-integration-20261104/ros-contract-validation.json)，原始summary位于 `runs/pick_stack_20261104_contract/summary.json`。源代码语法及交付包/仓库文件哈希另行核对。

## 生产只读前检

`runs/pick_stack_20261104_preflight/preflight.json`记录 `physical_actions_sent=0`。arm仍未能证明新鲜实测停稳，`arm_ready=false`，诊断 `fresh measured stationary interval unavailable`。读取到的夹爪样本为connected=true、feedback_valid=true、initialization=0、raw1000；没有自动初始化，样本详细字符串仍含先前TCP轮询失败信息，不能由单个字段推定系统整体正常。

因此本轮完成的是**执行接口代码和隔离ROS契约验证**，没有完成实物单枚抓放验收。现场当前位姿与失联后停稳不能从旧p1记录推定。

## 保留的实物边界

13mm棋厚、14mm棋盘厚、-10mm桌面和固定Home投影继续使用原正确值；码放在裸桌，不加棋盘厚。真实TCP、最低点、夹持/释放、支承和退出仍需量测。

原arm几何守卫没有动态持物或增长中堆叠碰撞体，本次未修改arm规划器填补该能力。实体模式是受监督demo：需要实际接触/持物/堆叠审查资料，并对每次持物移动和下降确认具体操作的路径包络。预览轨迹不绑定下一次执行Action内部重新规划的轨迹，不能把预览通过声称成动态碰撞已自动验证；任何人工回复也不会放宽arm原守卫。

首轮运行仍须先确认真实反馈、初始化状态、控制权与实物条件，在本聊天协调单枚实体授权；本次“接入代码”请求没有被解释成实体启动授权。完整协议和运行命令见[运行说明](../../tools/pick_stack/README.md)。
