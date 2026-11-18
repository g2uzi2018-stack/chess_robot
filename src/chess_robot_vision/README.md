# D405 相机节点

`chess_robot_vision` 提供 `vision_node`，负责 D405 彩色/深度采集、彩深对齐、畸变校正，以及图像、内参和帧信息发布。
棋子识别由 [感知包](../chess_robot_perception/README.md)负责；网页编码和显示由 [预览包](../chess_robot_preview/README.md)负责。

## 1 源码与配置入口

| 入口 | 用途与加载方式 |
| --- | --- |
| `src/node.cpp`、`src/calibration.cpp` | 相机通信、帧处理、发布及校正映射 |
| `launch/vision.launch.py` | 启动节点，默认加载安装目录中的 `config/d405.yaml` |
| [config/d405.yaml](config/d405.yaml) | 相机参数；每项用途、单位和特殊值见文件注释 |
| [config/99-chess-d405.rules](config/99-chess-d405.rules) | Linux udev USB 权限规则；目标目录、安装命令和加载方式见文件注释 |
| [config/large_images_fastdds.xml](config/large_images_fastdds.xml) | Fast DDS 的本机大图像传输配置；通过环境变量指定，加载方法见文件注释 |
| [FrameInfo.msg](../chess_robot_interface/msg/FrameInfo.msg) | 统一接口包中的帧信息字段定义与含义 |

配置文件不会因为放在 `config/` 中就自动生效。包构建会把配置安装到包的 share 目录，但不会把 udev 规则安装到 Linux 系统目录。
节点参数仅启动时读取，运行中修改会被拒绝；修改源码配置不会自动切换现场服务。

## 2 构建与启动

开发环境为 Jetson / ROS 2 Humble，使用本机 librealsense 2.58.1（RSUSB 用户态后端）及 OpenCV。
从项目根目录构建并加载开发安装：

```bash
cd /home/kuang/workspace/chess_robot
bash tools/build.sh --packages-up-to chess_robot_vision --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -Drealsense2_DIR=/home/kuang/.local/realsense-2.58.1/lib/cmake/realsense2
source tools/env.sh
```

下面的启动命令用于相机空闲时；已有相机进程占用 USB 时不要重复启动。
现场服务仍使用历史安装或冻结目录，开发构建不会切换它们；现场入口见[网页与相机操作](../../docs/operations/web-panels.md)。

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/install/dev/chess_robot_vision/share/chess_robot_vision/config/large_images_fastdds.xml"
ros2 launch chess_robot_vision vision.launch.py
```

可在 launch 命令末尾加 `config:=/absolute/path/d405.yaml` 选择另一份参数。前台运行时用 Ctrl+C 退出节点并释放相机。
`vision.launch.py` 本身只加载 YAML；上述环境变量在使用 Fast DDS 时选择 XML。现有相机预览服务脚本与叠加渲染脚本也会设置该变量。

直接执行 `ros2 run chess_robot_vision vision_node` 不会自动读取 YAML。代码默认值为 640×480、2 个 OpenCV 线程、曝光模式 keep；包内 YAML 选择 1280×720、4 个线程、手动曝光 25 ms 和增益 16。
当前感知节点只接受 1280×720 输入，调整相机尺寸时需同时核对下游配置与实现。

## 3 输出与数据含义

节点输出校正彩色图、对齐深度图、CameraInfo 和 FrameInfo。完整 Topic、类型、订阅方及 QoS 统一见 [ROS 接口](../../docs/ros_interfaces.md#3-相机感知和显示)。

- 彩色图为 BGR8；深度图为 32FC1，单位 m，深度空洞为 NaN。深度消息无需再次乘深度比例。
- 正常同帧图像、CameraInfo 和 FrameInfo 共用主机收帧时间戳；它不是曝光时刻。设备时间及时间域另存于 FrameInfo。
- 默认相机光学系为 `d405_color_optical_frame`，X 右、Y 下、Z 前；节点不发布相机到法兰或基座的手眼 TF。
- 深度先对齐到彩色图，再共同校正到针孔图平面。彩色用双线性重采样，深度用最近邻，避免跨物体边缘混合深度。
- 只有 `publish_raw=true` 且对应图像存在订阅者时，才发布该路未压缩图像；内参与帧信息仍随有效帧发布。这里的 raw 不表示未经校正的镜头原图。
- 相机不按照网页伪彩色的显示范围裁剪深度；网页显示设置由消费者负责。

## 4 运行状态与采样

启动日志打印实际相机、流配置和曝光读回值。D405 的曝光/增益作用于立体模块，同时影响彩色和深度。
未指定 serial 时要求恰好连接一台 D405；零台、多台或指定序列号不存在时启动失败。

无新帧超过 `stale_timeout_s` 时发布 `stream_ok=false`。单帧处理超过同一时限，或帧序、彩深配对检查失败时，该帧不作为有效图像发布，原因写入 FrameInfo。
异常首次及时记录，持续同原因异常每 2 秒汇总，原因变化和恢复立即记录；日志位置见[日志说明](../../docs/operations/logging.md)。

`max_frames=0` 持续运行；正数表示成功处理对应帧数后退出。
`evidence_dir` 非空时，在第 30 帧及指定末帧保存 `color.png`、`depth_z16.png`、`capture.json`，同名文件会覆盖，因此应使用独立的 `runs/<批次>/` 目录。
深度 PNG 保留 SDK Z16 原始单位，比例在 capture.json 中；这是单组采样，不是连续录像。

## 5 验证与边界

`test/test_calibration.cpp` 保留离线校正映射检查，构建与检查目录见[构建说明](../../docs/build-layout.txt)。
请求 30 fps 不代表已认证持续无丢帧；节点启动或服务 active 也不能单独证明图像持续更新。
当前时间戳和标定方式不构成运动中曝光同步、手眼绝对精度或抓取能力的验证。
