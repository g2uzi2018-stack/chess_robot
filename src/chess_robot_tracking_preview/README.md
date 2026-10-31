# C++ 棋子跟随编号预览绘制

该节点只绘制最新校正彩色图及带原始观测时间的实验跟踪结果，不做识别、规划、机械臂/夹爪控制，不发布生产TF。

输入：`/vision/color/image_rect`、`/arm/joint_states`、`/pose/xiangqi/detections`（实验JSON）。输出：`/pose/debug/image`与`/pose/xiangqi/preview_metrics`。

绘制速度随新相机帧推进，不等待推理。过期、运动或失配的观测不绘制坐标；视频继续。表面倾角为未验证的深度实验估计，不能用于动作，字面yaw未知。完整识别/跟踪、编号生命周期、现场启动、性能结果和Home运动证据见 [实验说明](../../tools/testing/xiangqi_tracking/README.md)。

构建（从工程根目录）：

```
source /opt/ros/humble/setup.bash
colcon build --base-paths src/chess_robot_tracking_preview
```

隔离绘制检查：

```
python3 src/chess_robot_tracking_preview/test/test_tracking_overlay.py \
  --binary /absolute/path/tracking_preview --output /absolute/NEW_test_directory
```

检查在ROS domain119自行启动独立进程，验证新鲜标记、过期/运动抑制以及视频持续；不连接真实机器人。
