# 机械臂六关节与夹爪控制面板

C++17 / ROS 2 Humble。`panel_node` 在 AGX 提供浏览器界面，通过现有
`/gripper/command` MoveGripper Action 下发指令，订阅 `/gripper/state`。
使用者目视判断激光笔是否夹牢，所有 MOVE 均为 `require_contact=false`。

## 六关节控制与设备切换

网页名称为“机器人控制台”，页头可切换机械臂和夹爪。切换只改变显示，不发送命令；切换设备会停止追加夹爪点动，切回后不会自动恢复。

六行 J1～J6 显示实测角度（°）。− / ＋ 根据当前反馈移动一步，默认 1°，步长可设 0.1～5°；按住不会重复运动。也可输入目标角度，再点击该行“执行”；输入或按回车本身不发送目标，状态刷新保留未提交输入。

`GET /api/state` 包含 `arm`、`arm_config`、`arm_command`；`GET /api/arm/state` 提供独立机械臂快照。`POST /api/arm/joint` 接收 `{request_id, server_id, joint_index, target_deg}`，joint_index=0～5。后端订阅并按原始时间戳匹配 `/arm/state` 与 `/arm/joint_states`，将角度转成 rad，经 `/arm/move_joints` MoveJoints Action 发送，其他关节保持当前实测值。

默认 `enable_arm_control=false`。启用必须同时满足 `enable_control=true`；实机还要求 `allow_hardware=true`。单次改变不超过 `arm_max_delta_deg`（上限 10°）；默认 profile 为 roundtrip_low，底层仍执行全部限位、几何、扭转及到位检查。参考有效、反馈新鲜且停稳、Action 可用才能发送。面板内夹爪和机械臂互斥，不自动 Home、初始化、使能、清错或重发。确定未执行的失败允许手动改值重试；执行结果不确定或超时保持锁定，迟到结果不能自动解锁。

2026-10-31 已完成现场部署，运行前缀为主工程 `runs/joint_panel_20261031_01/install`；配置与回滚副本位于该目录旁的 `deployment/`。机械臂由 `chess_robot_arm.service` 管理，面板和夹爪由 `chess_robot_console.service` 管理，原 `chess_robot_manual_panel.service` 已停止。下方现场部署段仅描述原夹爪版本历史。

## 网页入口

- [夹爪手动控制](http://192.168.30.26:8089)：当前页头“图像预览”跳转到8088。
- [相机预览](http://192.168.30.26:8088)：页头“夹爪控制”返回8089，均在当前标签页跳转。
- 页面跳转只切换网页，不会发送夹爪命令；离开控制页会结束连续点动的追加，最后已发出的步进仍可能走完。
- 当前链接面向此AGX的局域网地址；仅有8089的SSH转发不会同时转发8088，需要分别配置或直接使用局域网地址。
- 现场入口与维护见[双网页操作说明](../../docs/operations/web-panels.md)，实现和验收范围见[验证记录](../../docs/validation/manual-panel-20261031.md)。

## 操作

大按钮“张开/收拢”短按只动一步，持续按住350ms后连续点动；鼠标、触屏和空格/回车均支持。
松开、移出按钮、失去焦点、页面隐藏/关闭、断线、过期反馈或接触/故障后停止追加，需重新按住才继续。
每一步必须收到本次请求的成功结果和对应有效实际开度后才能发送下一步，不排队；
其他浏览器请求或服务重启也结束当前连续操作。
“指定开度”作为默认收起的辅助区，输入后点击发送。滑条/输入变化不会发命令。
0 为闭合方向，1000 为全开方向，均为原始单位，并非毫米；力设定百分比并非实测力。
默认 20% 力，上限 30%，超时 5 秒，步进 20 raw。现场先核对开合方向。
点动步长限1..100 raw，默认20；大按钮不再直接发送全闭0/全开1000。
配置中的open_position_raw用于辅助目标输入的初值，close_position_raw不作为大按钮目标。
松开后最后一条已发出的步进指令仍可能完成，不保证物理立即停止。夹持效果由使用者观察。

驱动返回码 12 表示检测到接触、未满足普通位置到位条件，面板保留此结果并提示目视检查。
不会把该结果自动改成“夹牢”。夹爪取消不可用，不提供停止按钮；断网、超时和关闭网页
均不保证停止。初始化有物理动作，只在点击时发送。

## 构建和模拟测试

必须使用同时包含 MoveArm、MoveJoints、MoveGripper 的统一接口包：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log_panel build --base-paths src/chess_robot_interfaces src/chess_robot_gripper src/chess_robot_panel \
  --build-base build_panel --install-base install_panel --executor sequential
source install_panel/setup.bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
ros2 launch chess_robot_gripper gripper.launch.py
# 另一终端，同样 source 和 ROS 环境
ros2 launch chess_robot_panel panel.launch.py
```

默认面板绑定 127.0.0.1:8089，仅允许模拟设备；启动并不发送初始化或开合。
通过 SSH 本地转发可访问：

```bash
ssh -p 6021 -L 8089:127.0.0.1:8089 kuang@119.45.181.86
```

浏览器打开 `http://127.0.0.1:8089`。

## 现场使用

复制 `config/panel.yaml` 为现场文件，把 `allow_hardware` 改为 true；局域网访问可将
`bind_address` 设为 AGX 的具体局域网地址（例如 192.168.30.26），不建议监听所有网卡。
面板与已运行夹爪均使用 `ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1`，不要重复启动夹爪驱动。
夹爪现场配置本身还需 `enable_commands=true`。面板不替驱动改参数或开工具电源；机械臂运动仅通过新增的显式关节操作发送。
HTTP 无账户体系，仅供可信本机/局域网，远程访问使用 SSH 转发，不发布公网端口。

```bash
ros2 launch chess_robot_panel panel.launch.py config:=/absolute/path/panel.site.yaml
```

## 行为和接口

- `GET /api/state`：反馈和年龄、允许操作、配置、最近请求及结果。
- `POST /api/gripper/initialize`：JSON `{request_id, server_id}`。
- `POST /api/gripper/move`：JSON `{request_id, server_id, position_raw, force_percent}`。
- POST 须有 `Content-Type: application/json` 和 `X-Panel-Request: 2`；旧版1返回403，要求刷新界面，以免旧按钮继续直接全开/全闭。
- server_id 取自本次进程的 GET；进程重启后旧页面的写请求被拒绝。
- 202 仅表示请求登记，结果由 Action 回调更新；不把 HTTP 成功当成执行成功。
- 同一时刻一个请求，无队列、无自动重试。进程内保存最多4096个请求ID，不淘汰重放记录，满后拒绝新请求。
- 相同ID相同参数不再发送，不同参数拒绝。页面刷新/重连不发送命令。
- 后端检查新鲜反馈（包含消息接收后的经过时间）、初始化、命令启用、忙闲及 Action 可用性。
- 发出后超时/通信异常或未收结果会锁定，不把客户端超时当物理停止。检查实物后按驱动规则恢复并重启面板。
- 不访问 RS485 或机械臂控制器端口；仅复用两个驱动的 ROS Action。

HTTP 使用 Boost.Beast，单独 I/O 线程与 ROS executor；输入体限制8KiB、连接有读写超时。
所有资源本地安装，无 CDN。前端以独立卡片组织，未来用明确类型的后端模块添加功能。
第一版不提供任意 topic/Action 发布器。

## 2026-10-31 原夹爪版本现场部署历史

代码位于夹爪工作区 `src/chess_robot_panel`，已在 AGX 编译到 `install_panel`。
现场配置在 `runs/manual_panel_20261031/`，手动服务为
`chess_robot_manual_panel.service`（临时 user service，未设置开机自启）。
局域网地址 `http://192.168.30.26:8089`；Windows 本机已建立
`127.0.0.1:8089 -> 192.168.30.26:8089` 的 SSH 转发。
部署初期曾观察到未初始化、开度1000；随后用户完成初始化及手动开合，不应将该初始快照当成当前运行状态。
以页面实时反馈为准。初始化期间位置超范围误锁定已修复；正常状态的范围检查与不自动重发仍保留。
Gemini 3.8 Flash提供布局初稿，已完成桌面/手机适配及点动交互回归；夹持效果仍由使用者目视判断。

```bash
systemctl --user status chess_robot_manual_panel
systemctl --user stop chess_robot_manual_panel
# 临时服务在停止/重启机器后可能被回收，可用以下命令重新启动（会保持前台）：
source /opt/ros/humble/setup.bash
source /home/kuang/workspace/chess_robot_gripper/install_panel/setup.bash
export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1
ros2 launch chess_robot_panel manual_panel.launch.py \
  panel_config:=/home/kuang/workspace/chess_robot_gripper/runs/manual_panel_20261031/panel.site.yaml \
  gripper_config:=/home/kuang/workspace/chess_robot_gripper/runs/manual_panel_20261031/gripper.site.yaml
```

该组合启动入口会启动一个夹爪驱动；已有夹爪驱动时只用 `panel.launch.py`。
消息时间采用同一 AGX 的 ROS 时钟，零时间戳或未来超过0.25秒的时间戳会禁用操作；
DDS 传输延迟计入反馈年龄。HTTP Host 仅允许绑定地址及本机地址，POST 另查同源。
