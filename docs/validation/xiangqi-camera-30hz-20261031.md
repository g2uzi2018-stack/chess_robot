# D405 720p 30Hz 发布与预览优化（2026-10-31）

保持1280×720、请求30fps、曝光25000us、增益16、当前彩深对齐、校正内参与手眼外参。
未启动、使能、移动、Home或重启机械臂/夹爪；只重启相机预览服务和只读叠加渲染服务。

## 测量与瓶颈

- USB链路为5Gbit/s，SDK独立原始采集10秒300帧，29.9946Hz、源帧号无跳跃。
- 实机列出的1280×720彩色BGR8及深度Z16模式均只有5/15/30fps；848×480等低分辨率才列有60/90fps。
- 独立SDK加对齐/校正（不经过ROS）约28.81Hz；对齐中位6.80ms，双图校正与深度转换中位26.49ms。
- 默认视觉ROS参与者共享内存段约512KiB，单帧彩色约2.64MiB、32FC1深度约3.52MiB；完成计算后仍出现大图交付损失。

## 已部署

- `src/chess_robot_vision/src/node.cpp`启动时预转换OpenCV remap映射；彩色线性插值与深度最近邻使用不同的映射表。
- `src/chess_robot_vision/config/d405.yaml`的`opencv_threads`由2改4；源和installed YAML保持一致。
- `src/chess_robot_vision/config/large_images_fastdds.xml`配置64MiB共享内存段、8MiB最大消息和loopback UDP发现。
- `src/chess_robot_preview/scripts/preview_service.sh`与只读`start_tracking_renderer.sh`在未显式设置时导出上述Fast DDS配置路径；不修改全局DDS环境或其他服务。
- 识别进程内部的三槽共享内存机制保持原设置；本次DDS调优用于ROS节点间的大图传输。

## 验证

- 4组1280×720随机/稀疏/真实保存图像，旧浮点映射与新映射的彩色和Z16输出逐像素差均为0。
- 交替计时各60次：浮点映射/2线程24.35ms，预转换映射/4线程14.19ms；单独改映射只节省约5%，主要收益来自校正并行。
- 相机包现有两项标定映射测试通过。
- C++独立订阅端：优化前FrameInfo约24.33Hz、彩色23.17Hz、深度23.0Hz；最终30秒四路各900帧，全部30Hz，900组stamp精确匹配，无无效帧、源帧号无跳跃，K与优化前相同。
- Windows局域网唯一JPEG序号实收21.36→29.72Hz；网页限制保持30Hz、1280×720。
- 中间配置（四线程/预转换映射、DDS最终调优之前）与LAN预览同时运行时，固定108帧识别回放18.61fps；这不是新版实机三维识别FPS。
- 当前现场图像很暗、缺少机械臂同步停稳反馈；没有用暗场验证识别准确率，仍等待正常光照和机械臂反馈后验证实机满负载。30秒结果不代表长期无丢帧认证。
- 各窗口时长不同，前后系统调度/负载可能变化。

## 证据与回退

- 本轮所有工具、配置、JSON与构建：`runs/xiangqi_camera_rate_20261031_01/`。
- `profiles.json`、`direct-probes.json`、`remap-benchmark.json`、`cpp-ros-before.json`、`cpp-ros-final.json`、`deployment.json`。
- 原二进制`vision_node-before`、源`node_before.cpp`、YAML`config-before.yaml`、启动脚本`preview-launcher-before.sh`和`renderer-launcher-before.sh`均在该目录。
- 回退时只停止视觉服务，恢复对应二进制/源码/YAML/启动脚本后启动视觉服务；不重启机械臂。
- 临时DDS试验的runtime drop-in已移除，实际部署从已安装包内读取配置；未启用开机自启，未提交或推送现场改动。

参考：
- https://docs.opencv.org/4.5.4/da/d54/group__imgproc__transform.html （convertMaps/remap）
- https://fast-dds.docs.eprosima.com/en/v2.6.11/fastdds/transport/shared_memory/shared_memory.html （共享内存段与大消息）
- https://www.intel.com/content/www/us/en/products/sku/229218/intel-realsense-depth-camera-d405/specifications.html （720p30规格；实机模式列表优先）
