# 预览节点

`chess_robot_preview` 是独立的 C++ / ROS 2 Humble 节点，从现有 ROS 图像 topic 接收数据，向局域网浏览器提供预览；不访问相机 USB，不控制机械臂和夹爪。
压缩、缩放与深度伪彩色均由预览节点负责，JPEG 只通过 HTTP 给浏览器，不另发 ROS 压缩话题。

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
- 默认包含彩色、深度和检测叠加图。检测项只订阅 `/pose/debug/image`，不执行检测或绘制叠加；无数据时显示“无数据”，未来发布图像后自动显示，过期/停发后恢复无数据。
- 只有点击的图像才订阅和编码。左侧选中项与右侧共用同一个浏览器 Blob，不额外下载缩略图。
- 同一浏览器切换选项时释放旧订阅；不同浏览器可以各看一路，同一路共享 JPEG 缓存。
- 暂停、页面隐藏、离开页面会释放查看需求；异常断开后 2 秒超时回收。隐藏页面仍可能低频读取元数据，不传输图像。
- 默认 30 fps、最大宽度 960 px、JPEG 质量 75。慢请求不堆积，浏览器仅一个图像请求在途。
- 无新图或断线时清除旧画面，显示等待状态；不会把冻结图像当实时图。
- 预览节点始终轻量监听 HTTP；无查看者时没有图像订阅或 JPEG 编码。相机本身的采集、对齐与校正仍是独立的基础开销。
- 相机节点按需发布未压缩 Image，内参与采集帧信息随处理频率发布；预览节点只处理当前查看的图像。

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
- `type: compressed` 可兼容外部已有的 `sensor_msgs/CompressedImage`，解码后仍由预览按本节点质量设置编码；默认三项均接收未压缩 Image。
- `mode: depth` 接收 **32FC1、单位 m** 深度；按配置距离范围伪彩色显示，无效或超出范围为黑色。
- `source_fps` 可填写已知输入帧率。彩色/深度源为 30；检测图默认 0（未知），使用预览限频时钟；低于目标的输入不会被补帧。
- 所有 JPEG 均由预览编码，默认质量 75、最大宽度 960；不再提供 `passthrough` 或相机侧预览压缩接口。
- 深度源为 32FC1 米制 Image，预览根据 `depth_min_m` / `depth_max_m` 生成伪彩色，默认 0.07 / 0.50 m；算法深度不受显示范围影响。
- 每路均为 SensorDataQoS、队列深度 2；上游需提供兼容 QoS。

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

- `GET /api/channels`：列表、是否存在发布者及接收/编码/响应字节统计，只取元数据不会开启图像。
- `GET /api/frame/{id}?client={browser_token}&after={sequence}`：按需选择、刷新租约；已有序号相同则最多等待 50 ms 新帧；200 JPEG，202 JSON `{"status":"no_data"}`（无新鲜有效图像），204 无新帧。无数据时清除旧图，降低重试频率，并继续等待未来发布者。
- `POST /api/release?client={browser_token}`：释放该浏览器的选择。
- 一个 token 最多选择一路；最多 16 个活跃查看 token、8 个并行 HTTP 请求。
- 响应不缓存，Web 资源全部随包安装，无 CDN 或云服务。

节点只用于局域网预览，当前没有账户系统或互联网发布。HTTP 的暂停/选择仅影响预览订阅，不会停止上游相机节点。
