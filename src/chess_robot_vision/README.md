# D405 相机节点

`chess_robot_vision` 保留原包名和 `vision_node` 可执行文件，职责为采集、彩深对齐、畸变校正、内参与帧信息发布。
圆候选检测、表面三维点计算、`/vision/candidates` 和检测叠加图已移除；后续棋子检测与位姿计算由独立位姿节点负责，本轮不创建该节点。

## 1 构建与启动

当前 AGX SDK 为 `/home/kuang/.local/realsense-2.58.1`，使用 RSUSB 用户态后端，没有修改内核。
官方源码 v2.58.1 压缩包 SHA256：`14409c3b810bf1508b87f46d47608a89018743b8a73d5855ab5d1ad18763fd8c`。
源码/构建日志位于 `runs/vision_deps/`（不随 Git 保存）。USB 权限规则为 `config/99-chess-d405.rules`。

```bash
cd /home/kuang/workspace/chess_robot
source /opt/ros/humble/setup.bash
colcon --log-base log_vision build --base-paths src/chess_robot_vision src/chess_robot_preview \
  --build-base build_vision --install-base install_vision --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_PREFIX_PATH=/home/kuang/.local/realsense-2.58.1
source install_vision/setup.bash
ros2 launch chess_robot_vision vision.launch.py
```

当前 AGX 已由 `chess_robot_preview.service` 组合运行相机和预览，不要再重复启动相机；该服务使用 ROS domain 90。
未指定 `serial` 时只接受恰好一台 D405，多相机必须指定序列号。参数均在启动时读取。

## 2 数据接口

所有输出使用 SensorDataQoS（best effort、volatile）。同一对图像、内参与帧信息共用 `header.stamp` 和 `header.frame_id`。

| Topic | 类型 | 内容 |
|---|---|---|
| `/vision/color/image_rect` | `sensor_msgs/Image` | 校正后的 BGR8 彩色图 |
| `/vision/aligned_depth/image_rect` | `sensor_msgs/Image` | 同图平面的 32FC1 深度，单位 m，SDK 空洞为 NaN |
| `/vision/color/camera_info` | `sensor_msgs/CameraInfo` | 校正图平面的 K/P 内参与零畸变系数 |
| `/vision/frame_info` | `chess_robot_vision/FrameInfo` | 帧号、SDK 时间戳/时间域、深度单位与采集有效状态 |
| `/vision/color/image_rect/compressed` | `sensor_msgs/CompressedImage` | 按需彩色 JPEG，质量 75 |
| `/vision/aligned_depth/preview/compressed` | `sensor_msgs/CompressedImage` | 按需深度伪彩色 JPEG，质量 75，仅供显示 |

图像/预览仅在有订阅者时生成对应发布数据；内参与帧信息随相机处理频率发布，默认 640×480、30 Hz。
`publish_raw=true` 允许发布上述未压缩的校正 Image，不表示未经校正的原始镜头图。

本机彩色标定为 Inverse Brown Conrady。节点先用 SDK 对齐深度到彩色图，再校正到共同针孔图平面；彩色双线性、深度最近邻重采样，保留空洞并避免混合物体边缘深度。
光学坐标为 `d405_color_optical_frame`：+X 右、+Y 下、+Z 前。没有发布相机到末端/基座的手眼 TF。

## 3 帧信息与后续位姿输入

- `header`：与同帧图像、CameraInfo 相同的主机收帧时刻；不是已校准的曝光时刻。
- `color_frame_number` / `depth_frame_number`（uint64）：SDK 提供的彩色/深度帧号。
- `device_timestamp_ms`（float64）、`timestamp_domain`（string）：SDK 返回值及其时钟域；域可能为 Global Time，不能默认当作未经换算的设备计数器。
- `depth_scale_m`（float64）：原生 Z16 数值到 m 的换算系数；32FC1 topic 已经是 m，不要重复换算。
- `stream_ok`（bool）、`diagnostic`（string）：采集有效性；超时/错误时发布无效状态，不发布伪造观测或零坐标。

后续位姿节点的推荐输入为：**校正彩色图 + 对齐米制深度 + CameraInfo**，按相同 header 时间戳匹配；FrameInfo 用于采集健康和时间追查。
目标检测、深度质量筛选、三维反投影、姿态估计及坐标变换均放在位姿节点，不能用预览 JPEG 的颜色当米制深度。
运动中的手眼标定仍需验证曝光时刻与机械臂反馈同步，当前只保留数据，未完成该标定。

`preview_min_depth_m` / `preview_max_depth_m` 默认 0.07 / 0.50，仅控制深度 JPEG 的显示颜色范围，不裁剪或过滤算法用的 32FC1 数据。

## 4 检查与采样

```bash
colcon --log-base log_vision test --base-paths src/chess_robot_vision \
  --build-base build_vision --install-base install_vision --packages-select chess_robot_vision \
  --return-code-on-test-failure

# 已运行相机时仅观察，不重复打开 USB
ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 python3 tools/testing/vision_smoke.py \
  /home/kuang/workspace/chess_robot/runs/camera-io-20261025 --observe-live

# 相机未运行时，有限帧采样后释放设备
ros2 run chess_robot_vision vision_node --ros-args \
  -p max_frames:=120 -p evidence_dir:=/absolute/path/capture
```

当前保留两项标定映射测试；旧圆检测/深度估计及其测试随功能一起移除，历史 8 项结果不代表当前存在这些算法。
有限帧采样保存 `color.png`、`depth_z16.png` 和 `capture.json`，均对应对齐并校正后的图平面，Z16 PNG 保留 SDK 原生单位。
不再生成候选数组或 `candidates.jpg`。
