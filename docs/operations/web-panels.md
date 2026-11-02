# 机器人控制台与局域网网页操作

## 1 页面与节点职责

| 页面 | 本现场地址 | 节点 / 服务 |
|---|---|---|
| 图像预览 | <http://192.168.30.26:8088> | `preview_node` / `chess_robot_preview.service` |
| 机器人控制台 | <http://192.168.30.26:8089> | `panel_node`、`gripper_node` / `chess_robot_console.service` |
| 检测叠加图 | 8088中的对应通道 | `tracking_preview`；识别与场景处理另为独立进程 |

普通导航、设备切换、页面刷新和GET状态请求不发送控制命令。离开控制页会停止追加夹爪点动，最后已发出的小步仍可能完成；页面关闭不构成物理急停。
换网段或设备时需检查跨页链接；仅转发8089不会同时转发8088。网页无账户体系，只用于可信本机/局域网，远程使用SSH转发。

## 2 工作区、安装与只读检查

夹爪节点原在同一Git仓库的 `feat/gripper-node-20261025` worktree `/home/kuang/workspace/chess_robot_gripper` 开发。它和主工作区共享Git历史，不是需要重新开发的第二套夹爪。
源码合入不自动切换现场节点；实际安装和配置以service、进程参数与发布记录为准。

```bash
systemctl --user show chess_robot_console.service chess_robot_arm.service \
  chess_robot_preview.service --property=ActiveState,SubState,MainPID,ExecStart
cat /home/kuang/workspace/chess_robot/.runtime/arm-install-prefix
```

本现场ROS环境为 `ROS_DOMAIN_ID=90`、`ROS_LOCALHOST_ONLY=1`，测试使用隔离domain。临时user service可能在停止或重启机器后被回收；不要假定旧unit或旧PID仍存在，也不要照历史目录再启动第二个驱动。

本现场网页绑定AGX的LAN地址，不必同时监听127.0.0.1。诊断时绕过HTTP代理直连实际绑定地址；本机回环连接拒绝或代理连接失败不等于节点故障：

```bash
curl --noproxy '*' --max-time 5 -fsS http://192.168.30.26:8088/ -o /dev/null
curl --noproxy '*' --max-time 5 -fsS http://192.168.30.26:8089/api/state
curl --noproxy '*' --max-time 5 -fsS http://192.168.30.26:8089/api/arm/state
```

GET成功仅证明相应HTTP路径可访问。夹爪连接和反馈正常但尚未初始化时，节点可以健康而MOVE前提仍不满足；不要为了证明健康自动初始化。机械臂同样分别检查反馈、参考、mode与busy。
相机运行时不重复打开USB；已有夹爪驱动时使用单独 `panel.launch.py`，不用组合 `manual_panel.launch.py`。
重启控制台会同时影响夹爪驱动；先核对空闲与夹持状态、当前脚本是否自动初始化，以及设备控制权。重启不等于物理停止或故障已恢复。

## 3 六关节控制

机械臂页显示J1～J6实测角度，−/＋根据最新反馈走一步，默认1°、可调0.1～5°；输入数值后需点击该行执行。按住不连续运动，一次仅变更一轴且不超过10°。

面板将原始匹配时间戳的实测关节和预期起点发送至 `/arm/move_joints`，底层继续检查反馈、参考、互斥、限位、几何、扭转、规划和实测到位。参考有效、反馈新鲜且停稳、Action可用才允许发送。超时或结果不确定保持锁定，迟到成功不会自动解锁；不自动Home、使能、清错或重发。

## 4 夹爪点动

1. 核对连接、真实设备标志、反馈年龄、初始化和忙闲。初始化会产生物理动作，已初始化时不应为了刷新状态再次初始化。
2. “收拢/张开”短按走一步，默认20 raw，步长1～100 raw；按住350ms后，只有前一步成功且收到对应有效开度才追加下一步，不排队。
3. 松开、移出、触控取消、失焦、页面隐藏、断线、反馈过期、接触/故障或其他客户端请求都会停止追加，恢复后需重新按住。
4. 指定开度、力值或滑条的编辑本身不发送命令；点击发送后仍需等待Action结果。

0为闭合方向，1000为全开方向；raw不是毫米。力设定默认20%、现场上限30%，不是实测牛顿值。普通位置命令使用 `require_contact=false`；接触反馈或到位不能证明棋子夹牢，实际夹持由操作者观察。

驱动初始化中位置参考尚未建立时发布未知开度，等待两个新鲜、合法、停稳的完成样本；正常状态的超范围检查保留。通信异常、发出后超时或关闭不会自动重放位置/力命令。
当前没有已验证的独立夹爪停止寄存器，Action取消明确拒绝；断网、超时或节点退出均不保证物理停止，不自动张开或断电。

## 5 接口、维护与访问

- 控制台只复用机械臂与夹爪的ROS接口，不直接访问控制器或RS485。
- `/gripper/command`是MoveGripper Action，`/gripper/state`为状态topic。协议、返回码及反馈年龄见[夹爪包](../../src/chess_robot_gripper/README.md)。
- `GET /api/state`与`GET /api/arm/state`只读；POST需要同源检查、本次server_id与`X-Panel-Request: 2`。HTTP 202仅表示请求登记，不代表实体成功。
- preview启动时缓存安装网页；网页文件改变后需在正常维护窗口加载新版本。`chess_robot_preview.service`包含相机与预览，重启会影响采集，不能当作普通网页刷新。
- 参数与本次安装入口见[panel包](../../src/chess_robot_panel/README.md)，机械臂Home/历史恢复见[日常操作](arm-quickstart.md)。

```bash
ssh -p 6021 -L 127.0.0.1:8088:192.168.30.26:8088 \
  -L 127.0.0.1:8089:192.168.30.26:8089 kuang@119.45.181.86
```

跨页链接仍可能指向现场LAN地址。远程设备无法路由到LAN时，分别打开本机8088/8089入口，或按实际网络调整链接。

## 6 六关节控制台部署验收（2026-10-31）

本节为历史冻结记录，不是当前安装选择指令。原前缀为 `runs/joint_panel_20261031_01/install`，配置、启动脚本及回退副本在同目录 `deployment/`；旧 `chess_robot_manual_panel.service` 已由 `chess_robot_console.service` 替代，不再提供旧服务的可复制启动步骤。

原机械臂在新鲜、参考有效、模式5且连续停稳时重启，保留干净存档与线缆历史。部署后两个机械臂Action可用，夹爪保持原开度；没有以本次部署执行实机运动验收。
该批次记录150项单元测试、6组机械臂loopback、9组面板HTTP/模拟ROS及9组夹爪接口回归通过；浏览器设备切换和关节/夹爪检查通过。真实LAN只读检查控制POST为0，不证明运动或夹持可靠性。

夹爪更早的11点实机收窄、首版只读结果分别见[收窄报告](../validation/gripper-sweep-20261025/REPORT.md)和[首版报告](../validation/gripper-node-20261025/REPORT.md)。初始化瞬时位置1074/1925导致旧驱动误锁定的修复保留在源码和历史证据中，不把当时开度当成当前实物状态。

原夹爪面板、初始化瞬时值修复与浏览器验证见[10月31日冻结记录](../validation/manual-panel-20261031.md)。
