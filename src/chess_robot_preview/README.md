# 预览节点

`chess_robot_preview` 是独立的 C++ / ROS 2 Humble 节点，从现有 ROS 图像 topic 接收数据，向局域网浏览器提供预览；不访问相机 USB，不控制机械臂和夹爪。

## 1 启动

```bash
cd /home/kuang/workspace/chess_robot
source /opt/ros/humble/setup.bash
source install_vision/setup.bash

# 相机已运行：仅启动预览节点
ros2 launch chess_robot_preview preview.launch.py

# 相机未运行：一起启动相机与预览节点（不要重复占用相机）
ros2 launch chess_robot_preview camera_preview.launch.py
```

本机局域网地址：[http://192.168.30.26:8088](http://192.168.30.26:8088)。监听地址默认只绑定 AGX 的这个 LAN 网卡。
单独启动预览节点时，相机节点和预览节点须处于同一 ROS domain。

本次已安装并启动用户服务 `chess_robot_preview.service`，包含相机与预览节点，使用 `ROS_DOMAIN_ID=90`、`ROS_LOCALHOST_ONLY=1`。未启用开机自启。
服务运行期间不要再用上述命令重复启动相机。查看和管理本实例：

```bash
systemctl --user status chess_robot_preview.service
systemctl --user stop chess_robot_preview.service
systemctl --user start chess_robot_preview.service
ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 ros2 topic list
```

## 2 面板行为与资源

- 左侧为可扩展的图像列表，右侧为选中图像的大图；首次打开不选择、不订阅图像。
- 只有点击的图像才订阅和编码。左侧选中项与右侧共用同一个浏览器 Blob，不额外下载缩略图。
- 同一浏览器切换选项时释放旧订阅；不同浏览器可以各看一路，同一路共享 JPEG 缓存。
- 暂停、页面隐藏、离开页面会释放查看需求；异常断开后 2 秒超时回收。隐藏页面仍可能低频读取元数据，不传输图像。
- 默认 30 fps、最大宽度 960 px、JPEG 质量 75。慢请求不堆积，浏览器仅一个图像请求在途。
- 无新图或断线时清除旧画面，显示等待状态；不会把冻结图像当实时图。
- 预览节点始终轻量监听 HTTP；无查看者时没有图像订阅或 JPEG 编码。相机本身的采集和识别仍是独立的基础开销。
- 相机节点的彩色/深度 ROS 图像改为有订阅者才转换发布；检测 JPEG 有订阅者才编码，默认跟随 30 Hz 相机输出。识别结果和内参保留原有频率。

预览开销随选中图像尺寸、帧率、编码质量及查看者数量变化。多看一路会增加那一路的处理和传输；同一路多个查看者不重复编码，但各自占用网络流量。

## 3 增加图像

编辑 `src/chess_robot_preview/config/channels.yaml`，在 `channels` 末尾增加一条并重启节点即可，不改前端。

```yaml
  - id: another_camera
    title: 另一张图
    description: 简短说明
    topic: /another/image
    type: image
    mode: color
```

- `id` 唯一，仅英文字母、数字、下划线、短横线。
- `type: image` 接收 `sensor_msgs/Image`，彩色模式通过 cv_bridge 转 BGR8。
- `type: compressed` 接收 `sensor_msgs/CompressedImage`，解码后按预览设置缩放编码。
- `mode: depth` 接收 **32FC1、单位 m** 深度；按配置距离范围伪彩色显示，无效或超出范围为黑色。
- `source_fps` 可填写已知的上游配置帧率。默认三路 D405 图像均为 30；当输入帧率不高于预览目标时逐帧处理，避免二次限频丢帧。未知来源可省略（默认 0），使用预览限频时钟。
- `passthrough: true` 允许尺寸未超限的 JPEG 直接转发，避免再次编码；此时 JPEG 质量由上游决定。默认三路使用相机侧按需压缩，彩色和深度预览质量 75，检测叠加图质量 85。
- 默认深度预览是相机生成的伪彩色 JPEG，颜色范围由相机的 `min_depth_m` / `max_depth_m` 决定；原始 32FC1 米制深度 topic 仍可供算法使用。自行配置 `type: image`、`mode: depth` 时使用预览 YAML 的深度颜色范围。
- 每路均为 SensorDataQoS、队列深度 1；上游需提供兼容 QoS。

配置是启动时读取。可用自定义文件：

```bash
ros2 launch chess_robot_preview preview.launch.py preview_config:=/absolute/path/channels.yaml
```

## 4 构建与接口

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log_vision build \
  --base-paths src/chess_robot_vision src/chess_robot_preview \
  --build-base build_vision --install-base install_vision --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_PREFIX_PATH=/home/kuang/.local/realsense-2.58.1
```

- `GET /api/channels`：列表及订阅/编码/响应字节统计，只取元数据不会开启图像。
- `GET /api/frame/{id}?client={browser_token}&after={sequence}`：按需选择、刷新租约；已有序号相同则最多等待 50 ms 新帧，避免采样时钟抖动丢帧；200 JPEG，202 等待源，204 无新帧。
- `POST /api/release?client={browser_token}`：释放该浏览器的选择。
- 一个 token 最多选择一路；最多 16 个活跃查看 token、8 个并行 HTTP 请求。
- 响应不缓存，Web 资源全部随包安装，无 CDN 或云服务。

节点只用于局域网预览，当前没有账户系统或互联网发布。HTTP 的暂停/选择仅影响预览订阅，不会停止上游相机节点。
