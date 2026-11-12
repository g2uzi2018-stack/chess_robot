# 预览职责独立与无数据占位验证

日期：2026-10-25（Asia/Taipei）。

## 1 最终职责与接口

- 相机 `chess_robot_vision / vision_node` 只负责 D405 采集、彩深对齐、畸变校正，发布校正 BGR8 彩色 Image、32FC1 米制深度 Image、CameraInfo 和 FrameInfo。
- 已删除相机侧 JPEG 编码、两路 CompressedImage 发布者及深度显示范围参数；`/vision/candidates` 和旧相机检测图仍无发布者。
- `chess_robot_preview / preview_node` 按查看需求订阅未压缩图像，独立完成缩放、深度伪彩色和 JPEG 压缩；JPEG 只通过 HTTP 返回浏览器，不发布 ROS 压缩话题。
- 默认三张卡片：彩色 `/vision/color/image_rect`、深度 `/vision/aligned_depth/image_rect`、检测叠加 `/pose/debug/image`，输入均为 `sensor_msgs/Image`。
- 检测叠加项只展示未来位姿节点产生的图像，不执行检测或叠加绘制。无发布者、无有效帧或图像过期时返回 HTTP 202 `{"status":"no_data"}`，前端清除旧图并显示“无数据”；恢复新图后自动显示。
- `GET /api/channels` 新增 `has_publisher`、`received_frames`，只读取元数据不会开启图像订阅。存在发布者不代表有可显示的新鲜图像。
- 首次打开及未点击卡片不订阅、不编码、不下载图像；选中项的缩略图与大图共用 Blob。同路多浏览器共享编码缓存，切换/暂停释放旧需求，异常断开后 2 秒回收。
- 默认 30 fps、JPEG 质量 75、限宽 960 px；深度显示范围由预览配置 `depth_min_m` / `depth_max_m` 控制，默认 0.07 / 0.50 m，不影响算法数据。
- 外部已有 CompressedImage 可按配置兼容接入，解码后统一由预览编码；已移除 JPEG 直接透传选项，默认相机链路不使用压缩 ROS 输入。

## 2 验证结果

- 相机及预览包构建通过；两项相机标定映射测试通过，CTest 包装汇总为 3 tests / 0 errors / 0 failures。
- 真实相机观察通过：收到彩色 177、深度 183、内参 184、帧信息 184 条，177 组四路同时间戳数据；深度单位约定、空洞 NaN、标定矩阵和帧号检查通过。
- FrameInfo 接收年龄 P99 约 23.8 ms，首次有效帧等待约 2.79 s，后续无无效采集状态；两个旧相机压缩话题和旧候选/检测图话题发布者均为 0。
- HTTP/ROS 集成 7 项检查通过：初始无订阅/无编码、两路有效 JPEG、无来源检测卡片返回 no_data、多客户端共享、显式释放及租约到期。
- 合成独立 topic 的 8 项恢复检查通过：配置新增、无发布者、新来源出现、停止出帧、拒绝旧时间戳、恢复、发布者退出及租约回收。没有向生产 `/pose/debug/image` 注入测试图。
- 前端 JavaScript 语法检查通过；完整浏览器布局、点击和 GPU 绘制的视觉验收受浏览器自动化工具限制，本报告不据此声称已经完成视觉验收。

## 3 局域网帧率与资源

Windows → AGX `192.168.30.26:8088`，高精度 QueryPerformanceCounter 计时，每路约 8 秒；单请求在途，统计不同 JPEG 序号。采样时共 2 个查看连接，结果随场景、并发和系统负载变化。

| 当前选项 | 不同 JPEG 实收 fps | ROS 接收/编码 fps | 单连接 JPEG MiB/s | 预览 CPU（单核=100%） | 预览 RSS MiB |
|---|---:|---:|---:|---:|---:|
| 彩色 | 29.73 | 30.11 | 1.00 | 16.7% | 68.2 |
| 深度 | 29.20 | 29.36 | 0.50 | 53.7% | 71.3 |

相机进程另约 112.5～112.6% 单核 CPU、105.4～106.2 MiB RSS。此测量为 HTTP 实收速率，未测浏览器 GPU 绘制帧率；接近目标 30 Hz，不是硬实时保证。未修改全局网络参数或 ROS 中间件配置。

## 4 证据与运行

- `camera-io-smoke.json`：真实相机四路输出与旧话题零发布者。
- `preview-regression.json`：按需加载、无数据检测占位、多客户端与释放。
- `source-loss.json`：缺失、过期、停发、退出与恢复。
- `lan-preview.json`：局域网不同 JPEG 实收、资源及带宽。
- 代码入口：`src/chess_robot_vision/README.md`、`src/chess_robot_preview/README.md`；对应测试保存在 `tools/testing/`。
- 原始构建和本轮运行日志：AGX `runs/preview-owned-20261025/`。测试程序的部分固定输出目录沿用旧路径，本文件夹保存本轮 JSON 快照。
- 当前用户服务 `chess_robot_preview.service` 已运行，ROS domain 90，ROS 数据限定本机，未启用开机自启。面板地址 `http://192.168.30.26:8088`，已有页面刷新后可见三张卡片。

独立位姿节点、棋子实物检测/定位误差、手眼/TCP 标定及运动时间同步仍待实现和验收。本轮未修改机械臂或夹爪控制，进行中的机械臂改动不包含在本次提交。
