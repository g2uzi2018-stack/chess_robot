# 局域网网页入口与手动夹爪操作

更新：2026-10-31（Asia/Taipei）。现场为 Jetson AGX Orin / Ubuntu 22.04 / ROS 2 Humble。

| 页面 | 局域网地址 | 页头跳转 | 节点 / 服务 |
|---|---|---|---|
| 图像预览 | <http://192.168.30.26:8088> | 夹爪控制 → 8089 | `preview_node` / `chess_robot_preview.service` |
| 夹爪手动控制 | <http://192.168.30.26:8089> | 图像预览 → 8088 | `panel_node`、`gripper_node` / `chess_robot_manual_panel.service` |

两个入口在**当前浏览器标签页**互相跳转。普通点击导航不会发送夹爪控制命令；
离开预览会释放图像查看需求，离开控制页会结束连续点动的追加。
最后已经发出的步进仍可能完成，页面切换不构成物理急停。
链接使用本现场AGX的局域网地址；换网段/设备需同步修改两个页面的链接。

## 工作区与版本边界

- 主工作区：`/home/kuang/workspace/chess_robot`，`main` 基于 `9076b6d`；包含相机、preview、机械臂及手眼标定。
- 夹爪工作区：`/home/kuang/workspace/chess_robot_gripper`，`feat/gripper-node-20261025` 基于 `215d07b`。
- 新增 `src/chess_robot_panel/`、初始化修复和本次网页/文档仍是本地未提交改动，**不等于已推送或已合入main**。
- 运行环境均为 `ROS_DOMAIN_ID=90`、`ROS_LOCALHOST_ONLY=1`；测试使用隔离domain89，勿混用。
- 两个服务均未设置开机自启；手动面板是临时user service，停止或重启机器后可能被回收。

## 手动点动

1. 观察真实设备模式、连接及反馈。已经初始化时不用再次初始化；初始化会产生物理动作。
2. “收拢/张开”短按只走一步；默认20 raw，步长可调1..100 raw，细调可用5或10。
3. 按住超过350ms，待当前步进成功且收到对应真实开度后，再发送下一步，不排队。
4. 松开、移出按钮、触控取消、窗口失焦、页面隐藏/离开、断线或反馈过期都会停止追加。
   接触/故障、其他客户端请求及服务重启也会结束当前连续操作，恢复后需重新按住。
5. “指定目标开度”位于默认折叠区；调整输入或滑条不会自动动作，必须点击发送。

原始开度0为闭合方向、1000为全开方向，raw不是毫米；力设定20%起，默认现场上限30%，并非实测牛顿值。
是否夹住激光笔由操作者目视确认；普通位置命令使用 `require_contact=false`。
接触反馈或到位反馈不能直接证明夹牢。真实夹指TCP、毫米开口、实际力及自动棋子抓取闭环仍未完成验收。

## 服务、配置和恢复

```bash
systemctl --user status chess_robot_preview.service chess_robot_manual_panel.service
```

preview源代码在主工作区，安装网页在 `install_vision/chess_robot_preview/share/chess_robot_preview/web/`。
手动面板源代码在夹爪工作区，安装在 `install_panel/`，现场配置在 `runs/manual_panel_20261031/`。
相机服务运行期间不要重复启动相机；已有夹爪驱动时不要重复启动组合夹爪launch。

```bash
# 手动面板临时服务被回收后，前台重新启动；确认没有已有夹爪实例。
cd /home/kuang/workspace/chess_robot_gripper
source /opt/ros/humble/setup.bash
source install_panel/setup.bash
export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1
ros2 launch chess_robot_panel manual_panel.launch.py \
  panel_config:=/home/kuang/workspace/chess_robot_gripper/runs/manual_panel_20261031/panel.site.yaml \
  gripper_config:=/home/kuang/workspace/chess_robot_gripper/runs/manual_panel_20261031/gripper.site.yaml
```

“驱动禁止命令”可能是只读配置，也可能是故障锁定；先读实时detail/最近结果，不能直接归因于配置。
本次初始化误锁定：初始化中位置曾报告1074/1925，旧驱动错误套用正常0..1000范围。
现于初始化状态2时不读取未建立参考的位置，发布未知开度；等两次新鲜、已初始化、到位且合法开度后确认成功。
初始化完成后的严格范围检查及故障锁定保留。结果不确定时，先检查实物是否停止、夹持状态，再按驱动要求恢复；不自动重放命令。

控制接口是 `/gripper/command` **MoveGripper Action**，状态为 `/gripper/state` topic。
浏览器命令POST需要 `X-Panel-Request: 2`，旧版1被拒绝并要求刷新页面，以免旧按钮直接全开/全闭。
驱动没有已验证的立即停止接口，松开只能停止追加，最后一小步可能继续完成。

## 远程访问

局域网设备直接访问上述地址。SSH方式若需两个网页都可达，应同时转发两端口：

```bash
ssh -p 6021 -L 127.0.0.1:8088:192.168.30.26:8088 -L 127.0.0.1:8089:192.168.30.26:8089 kuang@119.45.181.86
```

当前跨页链接仍指向现场LAN地址；远程设备不能路由到LAN时，应分别打开本机8088/8089入口，
或先调整跨页链接配置。仅转发8089并不会让8088同时可访问。网页没有账户系统，仅用于可信本机/局域网。

## 证据与边界

已完成AGX编译、20项夹爪单元测试、13组夹爪ROS集成检查、初始化瞬时值回归、9组面板HTTP/ROS集成检查、
20项点动浏览器检查、五种屏宽布局检查。数值来自本次各阶段测试，不等于实机长按精度或夹持可靠性验收。
2026-10-25已有20%力设定下1000→0的11点实机收窄报告；本次用户进行了初始化和手动开合，驱动返回过到位/接触反馈。
本次导航改动不发送夹爪命令、不修改点动脚本。preview会在进程启动时缓存网页，
因此更新后短暂重启了 `chess_robot_preview.service`（包含相机和预览）以加载新链接；夹爪服务未重启。
已验证1440/390/320宽度下同标签页往返跳转，浏览器无运行错误，夹爪控制POST数量为0。
