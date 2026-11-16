# ROS 节点与接口

本页描述 Jetson **工作区源码**的默认接口；现场冻结服务可能采用不同实现。字段以接口源文件为准。
默认业务 domain 由启动入口设置为 90；localhost 设置为 1。固定绝对话题名不会随节点 namespace 自动改变；适配时用明确 remap。

## 1 节点和启动入口

| 默认节点名 | 包 / 职责 | 开发入口 |
| --- | --- | --- |
| `/chess_robot_arm` | chess_robot_arm；Nova5 通信、运动执行、几何和绕转状态 | `ros2 launch chess_robot_arm arm.launch.py`；默认配置为只读、暂停，现场覆盖见机械臂操作 |
| `/gripper_node` | chess_robot_gripper；AG95 通信、开度与夹持动作 | `ros2 launch chess_robot_gripper gripper.launch.py`，核对配置选择 |
| `/vision_node` | chess_robot_vision；D405 基础 RGB-D、校正和帧信息 | `ros2 launch chess_robot_vision vision.launch.py` |
| `/preview_node` | chess_robot_preview；按需图像编码和 HTTP 预览 | `ros2 launch chess_robot_preview preview.launch.py`；组合 camera_preview.launch.py 还启动相机 |
| `/xiangqi_perception` | chess_robot_perception；RGB-D 棋子测量 | `scripts/start_perception.sh`，路径相对包目录 |
| `/xiangqi_scene` | chess_robot_perception；基座变换、跟踪、稳定观察 | `scripts/start_scene.sh`；start_live_tracked.sh 管理以上两个进程 |
| `/xiangqi_tracking_preview` | chess_robot_tracking_preview；识别结果叠加 | `ros2 run chess_robot_tracking_preview tracking_preview` |
| `/xiangqi_scan_controller` | chess_robot_tasks；观察点扫描调度 | `ros2 run chess_robot_tasks scan_controller --config <JSON> --output <新目录>` |
| `/xiangqi_pick_stack_controller` | chess_robot_tasks；抓放、排一行及准备流程使用的 ROS 客户端 | `ros2 run chess_robot_tasks pick_stack ...` 或包 scripts/run_speed_demo.sh |
| `/panel_node` | chess_robot_panel；网页设备控制 | panel.launch.py；manual_panel.launch.py 还启动夹爪 |
| `/chess_robot_handeye` | chess_robot_handeye；标定采样、求解、验证 | handeye.launch.py；模板默认 validate，无 ROS I/O |

模型包、接口包和扭转库不要求各自启动节点。chess_robot_validation 是按需只读/仿真诊断工具包，入口见其 README；不会随业务启动自动运行。

## 2 设备和运动接口

下表自定义类型的共同前缀为 `chess_robot_interfaces`。结构与逐字段说明见 [接口源码](../src/chess_robot_interfaces)。

| 完整名称 | 类别 / 类型 | 提供方 → 主要使用方 | 含义和完成条件 |
| --- | --- | --- | --- |
| `/arm/state` | Topic / msg/ArmState | arm → scene、任务、panel、handeye | 实测状态、TCP、历史资格和故障；按配置频率发布 |
| `/arm/joint_states` | Topic / sensor_msgs/msg/JointState | arm → scene、任务、panel、handeye、叠加 | 实测关节，位置 rad、速度 rad/s；按 name 对齐，不能猜数组顺序 |
| `/arm/move` | Action / action/MoveArm | arm → 扫描、抓放、handeye | 支持规划/执行；按结果和实测终点判断完成；取消回执不是物理停稳 |
| `/arm/move_joints` | Action / action/MoveJoints | arm → panel | 关节目标，沿用驱动的参考、执行和故障条件 |
| `/arm/confirm_grasp` | Service / srv/ConfirmGrasp | arm → 需要夹持确认的客户端 | 对当前待确认操作提交匹配的抓取结果 |
| `/arm/initialize_reference` | Service / srv/InitializeReference | arm → 参考建立操作 | 显式建立参考，不是日常更新 Home |
| `/arm/restore_reference` | Service / srv/RestoreReference | arm → 历史恢复操作 | 根据连续审计资料恢复；失败原因在响应中 |
| `/arm/restore_checkpoint` | Service / srv/RestoreCheckpoint | arm → 设备准备流程 | 恢复可信停稳存档；实际使能行为取决于现有配置 |
| `/gripper/state` | Topic / msg/GripperState | gripper → panel、抓放 | 实际开度、运动/夹持及故障；开度原始单位见源定义 |
| `/gripper/command` | Action / action/MoveGripper | gripper → panel、抓放 | 初始化、开合等操作；成功和取消以节点反馈及结果为准 |

arm 的 `test_fixture=true` 会把上述 `/arm/` 前缀改为 `/simulation/arm/`。运动长度 m、角度 rad，TCP 位于 base_link，四元数按 XYZW。
机械臂和夹爪状态使用可靠 Topic；具体队列、周期和配置入口见 node.cpp 与包内 config。实体驱动的通信、拒绝和恢复条件见对应驱动和配置说明。

## 3 相机、感知和显示

| 完整名称 | 类型 | 提供方 → 使用方 | 数据含义 / QoS |
| --- | --- | --- | --- |
| `/vision/color/image_rect` | sensor_msgs/msg/Image | vision → perception、preview、tracking_preview、handeye | 校正后的 bgr8 基础图像；SensorDataQoS |
| `/vision/aligned_depth/image_rect` | sensor_msgs/msg/Image | vision → perception、preview | 对齐彩色图的 32FC1 深度，单位 m；SensorDataQoS |
| `/vision/color/camera_info` | sensor_msgs/msg/CameraInfo | vision → perception、handeye | 对应校正图内参；SensorDataQoS |
| `/vision/frame_info` | chess_robot_vision/msg/FrameInfo | vision → perception、handeye | 同帧序号、设备时间和流状态；SensorDataQoS |
| `/perception/xiangqi/measurements` | std_msgs/msg/String，JSON | perception → scene | 相机测量协议 v2；BEST_EFFORT、depth=3 |
| `/perception/xiangqi/roi_hint` | std_msgs/msg/String，JSON | scene → perception | 下游建议的图像 ROI；不是设备原始数据或动作命令 |
| `/pose/xiangqi/detections` | std_msgs/msg/String，JSON | scene → 任务、tracking_preview | 当前测量、跟踪及基座资格；可靠、depth=10 |
| `/pose/xiangqi/slow_debug_image` | sensor_msgs/msg/Image | perception → 按需调试显示 | 识别调试图，SensorDataQoS |
| `/pose/debug/image` | sensor_msgs/msg/Image | tracking_preview → preview | 叠加图，SensorDataQoS depth=1 |
| `/pose/xiangqi/preview_metrics` | std_msgs/msg/String，JSON | tracking_preview → 诊断 | 显示指标，不能作为识别完成率 |

相机四类消息共享配对时间戳，表示**主机收到帧**，不是曝光时刻；设备时间另在 FrameInfo 中保留。
相机光学系为 d405_color_optical_frame（X 右、Y 下、Z 前）。识别中心是可见顶面中心，不自动等于抓取 TCP。
消息校验和 schema 以 [measurement_contract.py](../src/chess_robot_perception/chess_robot_perception/measurement_contract.py) 为准；该文件只定义当前协议，不重复复制字段表。
preview 按 channels.yaml 配置订阅，并提供 HTTP，不负责 YOLO。当前绝对名称由源代码声明，参数和 remap 需在启动入口明确传入。

## 4 观察与任务交互

| 名称 | 类别 / 类型 | 提供方与使用方 | 行为 |
| --- | --- | --- | --- |
| `/pose/xiangqi/observation/request` | Topic / String JSON | 客户端 → scene | collect/query/consume 等请求；包含请求标识 |
| `/pose/xiangqi/observation/response` | Topic / String JSON | scene → 请求客户端 | 对应请求结果，可靠 depth=32；没有实现成 ROS Service |
| `/pose/xiangqi/observation/status` | Topic / String JSON | scene → tracking_preview、诊断 | 当前观察阶段；可靠、TRANSIENT_LOCAL depth=1 |
| `/pose/xiangqi/observation/snapshot` | Topic / String JSON | scene → tracking_preview、消费者 | 窗口满足现行 Policy 后的快照；可靠、TRANSIENT_LOCAL depth=1 |
| `/scan/xiangqi/status`、`/scan/xiangqi/result` | Topic / String JSON | scan_controller → 诊断/调用方 | 当前阶段和结果文件，可靠、TRANSIENT_LOCAL depth=1 |
| `/pick_stack/status`、`/pick_stack/confirmation_request` | Topic / String JSON | 抓放主控 → 交互客户端 | 任务状态及当前人工确认请求；可靠、TRANSIENT_LOCAL depth=1 |
| `/pick_stack/confirmation` | Topic / String JSON | 交互客户端 → 抓放主控 | 按本次请求标识回应，不能重放历史回答 |
| `/chess_robot_handeye/abort` | Service / std_srvs/srv/Trigger | handeye → 操作者调用 | 请求终止；响应不证明实体已停止 |

稳定窗口和快照规则见 [stable_observation.py](../src/chess_robot_perception/chess_robot_perception/stable_observation.py) 及 [观察说明](vision/stable-observation.md)。
扫描按所选 config/scan JSON 的观察点、确认和截止条件采样；抓放按 config/pick JSON 调度，实际 arm/夹爪动作通过公开 Action。
手眼的 Topic 和 Action 名通过 handeye 配置指定，abort 使用私有名 ~/abort，随节点实例名称变化。

运行日志的位置、统计周期和异常汇总见 [日志说明](operations/logging.md)。日志输出调整不改变本页 Topic、消息、QoS 或业务处理频率。

## 5 同步和核实

节点新增、删除、改名、职责、接口语义、启动或交互条件变化时，同次更新本页与受影响的模块说明；内部修改仍须核对本页。
使用 `ros2 node list`、`ros2 node info <节点>`、`ros2 topic list -t`、`ros2 topic info <话题> -v` 核对当前运行图。
使用 `ros2 interface show <类型>` 查看当前环境安装的定义。运行查询不能代替本页的约定；默认框架接口不逐条列出。
