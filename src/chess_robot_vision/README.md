# D405 相机节点

`chess_robot_vision` 保留原包名和 `vision_node` 可执行文件，职责为采集、彩深对齐、畸变校正、内参与帧信息发布。
圆候选检测、表面三维点计算、`/vision/candidates` 和相机侧检测图生成已移除；后续棋子检测与位姿计算由独立位姿节点负责，本轮不创建该节点。
相机只发布未压缩算法数据，不发布 JPEG/CompressedImage；网页压缩与深度伪彩色由预览节点处理。

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

图像仅在有订阅者时生成对应发布数据；内参与帧信息随相机处理频率发布，默认 1280×720、请求 30 Hz。
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

相机不按预览颜色范围裁剪或过滤 32FC1 深度；显示范围由消费者自行设置。

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

## 5 当前分辨率与重新插拔复测（2026-10-30）

当前组合配置为 **1280×720、请求30 fps**，源和installed YAML同步；网页最大宽度设为1280，彩色、深度、检测叠加均已验证实际1280×720且序号持续增长。请求帧率不代表已认证无丢帧30 Hz。

用户保留约2m延长线重新插拔后，现有vision_node两次独立启动各完成60帧，独立SDK另完成40对。正式运行旁路观察得到129组同stamp图像/深度/K/FrameInfo，无stream invalid，源帧号严格增长；43次ChArUco检测74–88角点。预览持续运行，机械臂未被控制。详细证据见 [重新插拔复测](../../docs/validation/d405-replug-20261030.md)。

2026-10-29曾出现1280彩色UVC probe/commit STALL，见 [历史验证](../../docs/validation/d405-1280-20261029.md)。本次恢复不能区分接触改善与相机重新上电清除异常，也不能证明延长线长期可靠。历史Python对比统计的K/D传参更正保存在runs/camera_1280_20261029/errata.md，旧多帧RMS不应作为产品或分辨率优势证据。

切换时先stop组合服务、等待2秒、start，并检查真实图像更新；service active本身不能证明相机正常。848回退备份位于runs/camera_replug_20261030_01/camera-{source,installed}-before-deploy.yaml。独占USB探针必须先停止正式相机。

## 6 现场曝光控制（2026-10-31）

1280×720、请求30Hz保持不变。黑色桌垫下自动曝光导致浅色棋子过曝及明显噪声；本次对比自动与2/5/10/16/25ms、固定增益16后，采用启动参数 exposure_mode: manual、exposure_us: 25000.0（微秒）、gain: 16.0。D405彩色来自立体模块，曝光同时影响彩色和深度；光照变化后需重新观察。25ms档整帧非零深度约85.36%，10ms档约71.77%；这不是棋子定位精度。

exposure_mode 支持 keep（代码默认，保留SDK现状）、auto、manual。所有参数仍仅启动时读取，现场YAML默认显式manual；SDK范围检查后写入，原始曝光/增益元数据附在FrameInfo.diagnostic。启动日志打印实际彩色流尺寸/帧率与曝光读回。不改变内参、校正、彩深对齐或手眼外参。更换光照时可改回auto并重启组合相机预览服务。

源码、构建、曝光对比、部署前二进制及YAML备份、同步采样和只读识别试验见 runs/xiangqi_recognition_20261031_02/。实验Python预览不属于正式C++位姿节点，不发布生产TF、不发动作。详细复现见 docs/validation/xiangqi-recognition-20261031.md。

## 7 720p30 发布与传输优化（2026-10-31）

保持1280×720、30fps、曝光25000us、增益16。预转换独立的彩色线性/深度最近邻remap表，OpenCV线程2→4；视觉启动脚本在未显式覆盖时使用`config/large_images_fastdds.xml`（Fast DDS共享内存段64MiB、最大消息8MiB、loopback UDP发现）。只作用于相机/预览和只读叠加渲染，不改变全局DDS或机械臂配置。

当前暗场/关臂环境，独立C++观察30秒获得900组同stamp RGB/深度/K/FrameInfo，四路30Hz、无无效或源帧跳跃，局域网叠加实收29.7Hz。4组720p输入校正前后彩色/Z16像素完全一致。开灯并恢复机械臂反馈后的实时三维识别满负载仍待验证；此结果不作长期无丢帧认证。详情、回退与原始证据见[本轮验证](../../docs/validation/xiangqi-camera-30hz-20261031.md)。
