# D405 视觉节点（第一阶段）

来源规格：[中国象棋夹取](https://app.notion.com/p/3ff9f9a4e09e81fca280e1f3e4c42140)。

本包使用 C++17 / ROS 2 Humble、librealsense 和 OpenCV，直接在 AGX 读取 USB D405。
当前检测器为 Hough 圆候选基线，**没有棋子身份分类模型**，输出 `unknown_circle`，也会检测到非棋子的圆形物体。
该阶段用于检查成像、深度和后续接口，不能视为完整棋子识别或夹取验收。

## 1 构建与启动

AGX 当前 SDK 位于 `/home/kuang/.local/realsense-2.58.1`，采用 RSUSB 后端。
官方参考：[Jetson 安装说明](https://github.com/realsenseai/librealsense/blob/v2.58.1/doc/installation_jetson.md)。
SDK 使用用户目录安装，没有改动内核；USB 设备需要 plugdev 组写权限。

本次 SDK 固定为官方 `v2.58.1`，源码压缩包 SHA256：
`14409c3b810bf1508b87f46d47608a89018743b8a73d5855ab5d1ad18763fd8c`。
构建开关为 `FORCE_RSUSB_BACKEND=ON`、`BUILD_EXAMPLES=OFF`、`BUILD_TOOLS=OFF`、
`BUILD_GRAPHICAL_EXAMPLES=OFF`、`BUILD_UNIT_TESTS=OFF`、`BUILD_ROSBAG2=OFF`、
`CHECK_FOR_UPDATES=OFF`、`BUILD_WITH_CUDA=OFF`；第三方依赖 nlohmann/json 为 SDK 指定的 v3.12.0。
源码与构建日志在工程的 `runs/vision_deps/`，不包含在 Git 中。

设备权限规则保存在 `config/99-chess-d405.rules`，仅匹配 D405：

```bash
sudo install -m 0644 src/chess_robot_vision/config/99-chess-d405.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=change --subsystem-match=usb --attr-match=idVendor=8086 --attr-match=idProduct=0b5b
sudo udevadm settle
```

```bash
cd /home/kuang/workspace/chess_robot
source /opt/ros/humble/setup.bash
colcon --log-base log_vision build --base-paths src/chess_robot_vision \
  --build-base build_vision --install-base install_vision \
  --executor sequential --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_PREFIX_PATH=/home/kuang/.local/realsense-2.58.1
source install_vision/setup.bash
ros2 launch chess_robot_vision vision.launch.py
```

参数在 `config/d405.yaml`，均为启动参数，运行期间修改会被拒绝。相机分辨率组合必须由设备支持；不自动降级。
未指定 `serial` 时仅接受恰好一台 D405，防止多相机时默默选错。
使用独立 build/install/log 目录，避免覆盖正在开发的机械臂包。

## 2 输出接口

所有 topic 使用 SensorDataQoS（best effort，volatile）；订阅者应匹配 QoS。
接入预览节点后，彩色和深度图仅在有订阅者时转换发布；检测 JPEG 仅在有订阅者时编码，`debug_fps` 默认随相机以 30 Hz 输出。
候选结果与内参仍按采集处理频率发布，`evidence_dir` 的本地采样行为保留。

| Topic | 类型 | 内容 |
|---|---|---|
| `/vision/color/image_rect` | `sensor_msgs/Image` | BGR8，校正到无畸变针孔图平面的彩色图 |
| `/vision/aligned_depth/image_rect` | `sensor_msgs/Image` | 同图平面的 32FC1 深度，单位 m，空洞为 NaN |
| `/vision/color/camera_info` | `sensor_msgs/CameraInfo` | 校正图平面内参，畸变系数为零 |
| `/vision/candidates` | `chess_robot_vision/PieceCandidates` | 本帧圆候选及可选表面三维点 |
| `/vision/debug/image/compressed` | `sensor_msgs/CompressedImage` | 标注候选的 JPEG，绿色深度有效，橙色无效 |
| `/vision/color/image_rect/compressed` | `sensor_msgs/CompressedImage` | 按需生成的校正彩色 JPEG，质量 75，随相机 30 Hz |
| `/vision/aligned_depth/preview/compressed` | `sensor_msgs/CompressedImage` | 按需生成的深度伪彩色 JPEG，质量 75；仅供显示，不是米制测量图 |

`header.frame_id=d405_color_optical_frame`：+X 右、+Y 下、+Z 前。
实机 D405 彩色流返回 **Inverse Brown Conrady** 畸变，不能直接把这组系数标为 ROS `plumb_bob`。
节点先用 SDK 将深度对齐到彩色图，再按实际标定生成逆映射，将两图校正到同一无畸变针孔平面。
彩色使用双线性重采样，深度使用最近邻，保留空洞并避免在物体边缘混合深度。
校正后保留光学轴方向；图像边界可能出现无效像素。
同一帧图像、内参和结果共用主机收到同步帧时的 ROS 时间戳，**不是曝光时间或已校准的硬件时钟**。
同时输出原始硬件时间戳、时间域、帧号以供追查。
手眼标定尤其是运动中采集，还需验证曝光时刻与机械臂反馈的时间对齐。

`surface_point` 是可见圆中心附近的表面点，不是棋子几何中心、TCP 或可执行抓取姿态。
深度使用中心内缩圆片的中位数，检查有效像素比例、样本数量、距离范围与 MAD；读取 SDK 实际深度单位，不能假定 Z16 固定为毫米。
深度不合格时 `depth_valid=false`，坐标为 NaN。
`label=unknown_circle` 不代表已确认棋子；候选索引也不是稳定追踪 ID。

若超过 `stale_timeout_s` 无帧，则持续发布 `stream_ok=false`、空候选列表。
处理超时的帧也丢弃。启动失败会非零退出；持续传感器故障需检查后重启。
消费者必须同时检查 stream_ok、depth_valid、结果年龄和目标类别；不能缓存最后一个坐标继续执行。
本包不发布 TF，不调用机械臂或夹爪，不伪造相机到 base_link 的外参。

## 3 验证与采样

```bash
colcon --log-base log_vision test --base-paths src/chess_robot_vision --build-base build_vision --install-base install_vision \
  --packages-select chess_robot_vision --return-code-on-test-failure
colcon test-result --test-result-base build_vision --verbose

# 独立 ROS domain，有限帧真实相机测试；只启动/停止本测试拥有的相机进程。
ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 python3 tools/testing/vision_smoke.py \
  /home/kuang/workspace/chess_robot/runs/vision-20261025

ros2 run chess_robot_vision vision_node --ros-args \
  --params-file src/chess_robot_vision/config/d405.yaml \
  -p max_frames:=120 -p evidence_dir:=/absolute/path/to/capture
```

`max_frames=0` 表示持续运行；正数用于有限帧 smoke test。
指定 `evidence_dir` 后第 30 帧及最后一帧保存 `color.png`、`depth_z16.png`、`candidates.jpg` 与 `capture.json`。
两张 PNG 均为对齐并校正后的图，深度 PNG 保留 SDK 的 Z16 单位，并非原始深度镜头图。
JSON 中包含深度单位、输出内参、原彩色畸变模型及系数、候选点；两次保存覆盖同一组文件。
测试用图和真实图的结果都只验证各自覆盖的条件，不能推断未出现的棋盘或棋子。

## 4 后续接入

1. 在实际棋盘、安装距离和照明下采集样本，确认遮挡、倾斜及有效深度覆盖。
2. 使用训练后明确版本和类别映射的模型替换圆候选检测器，验证检出率、误检和定位误差。
3. 标定相机相对末端的刚体变换、真实 TCP，并验证静态/运动时间同步。
4. 根据棋子高度、夹爪接触位置和桌面法向生成抓取姿态，经过任务节点和机械臂门禁执行。
