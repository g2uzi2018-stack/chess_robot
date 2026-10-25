# 中国象棋夹取视觉节点第一版

日期：2026-10-25（Asia/Taipei）

## 1 本轮范围

依据 Notion“中国象棋夹取”，在 Jetson AGX Orin 的 `/home/kuang/workspace/chess_robot` 增加 C++17 / ROS 2 Humble 包 `src/chess_robot_vision`。
第一阶段范围为 D405 同步彩深取流、相机内参、圆形候选、候选表面三维坐标与诊断图。
用户已明确本轮先完成节点，稍后摆放棋盘和棋子验收。

## 2 实现

- 五路 ROS 输出：彩色图、对齐深度、内参、候选数组、标注 JPEG。
- 实机彩色流为 Inverse Brown Conrady 畸变；彩色和深度按 SDK 标定转换到同一无畸变针孔图平面，输出 topic 为 `image_rect`，不会把逆畸变系数错误标为 ROS plumb_bob。
- 深度输出为 32FC1、单位 m，空洞 NaN；Z16 转换使用实际 SDK 深度单位。
- 使用中心圆片中位数、有效比例、样本量、距离与 MAD 检查深度。
- 候选仅为 `unknown_circle`，不宣称已识别到棋子身份；无效三维点为 NaN。
- 光学坐标系为 `d405_color_optical_frame`，+X 向右、+Y 向下、+Z 向前。
- 图像与结果共用主机收帧时间；另外保留硬件时间戳、时间域、帧号。没有完成曝光时刻对齐。
- 拒绝不匹配、重复或乱序帧；无帧超时发布 stream_ok=false 和空列表；参数仅启动时设置。
- 不发布未经标定的外参，不生成或执行机械臂、夹爪命令。

## 3 环境改动

- SDK：官方 librealsense v2.58.1，用户目录 `/home/kuang/.local/realsense-2.58.1`，RSUSB 后端。
- SDK 压缩包 SHA256：`14409c3b810bf1508b87f46d47608a89018743b8a73d5855ab5d1ad18763fd8c`。
- 未改内核。仅为 D405 的 USB ID 8086:0b5b 安装 plugdev 组访问规则 `/etc/udev/rules.d/99-chess-d405.rules`。
- 视觉使用 `build_vision`、`install_vision`、`log_vision`，测试使用独立 ROS domain 89 与 localhost。
- 本轮添加视觉包、独立测试工具和视觉报告，并补充三个视觉构建目录的 Git 忽略规则；未修改机械臂及共享接口文件。

## 4 验证结果

**构建与启动入口通过，8 项单元测试通过，180 帧真实相机 ROS smoke test 通过。**

相机序列号 `323622271784`，固件 `5.16.0.1`，USB `3.2`，分辨率 `640×480 @ 30 fps`，实际 SDK 深度单位约 `0.0001 m/unit`。

| 检查 | 本次结果 |
|---|---|
| C++ 构建、运行时库解析、launch 参数入口 | 通过 |
| 单元测试 | 8 项通过：5 项深度、1 项圆检测、2 项标定映射 |
| 真实相机处理帧数 | 180，正常退出并释放设备 |
| 候选、内参、JPEG 消息 | 各 180 条 |
| 彩色 / 深度消息 | 177 / 175 条（SensorDataQoS 允许丢帧） |
| 五路均收到的同时间戳组 | 173 组 |
| 有限深度像素比例中位数 | 91.915%（不等同于物理测距正确率或全部处于 7–50 cm） |
| 候选消息接收延迟 P50 / P99 | 21.5 / 31.6 ms（主机收帧到测试接收，不是曝光到结果延迟） |
| 圆候选观测 / 与深度图 Z 值复核 | 31 / 31 次，通过；不是 31 枚棋子或识别成功次数 |
| 最后保存帧候选数 | 0 |

标定往返测试采用本机读出的逆 Brown 参数，在全图网格检查校正映射与 SDK 反投影的一致性，误差阈值 0.01 px。
这验证软件数学一致性，不替代实物定位精度标定。
测试日志与采样在 AGX 的 `runs/vision-20261025/`；`smoke.json` 包含机器可读结果。

## 5 启动

```bash
cd /home/kuang/workspace/chess_robot
source /opt/ros/humble/setup.bash
source install_vision/setup.bash
ros2 launch chess_robot_vision vision.launch.py
```

完整构建、接口、采样和验证命令见 `src/chess_robot_vision/README.md`。代码位于 AGX 当前工程，并随本报告纳入 Git；版本以提交历史为准。
采样脚本为 `tools/testing/vision_smoke.py`。

## 6 尚未验收

实拍中棋盘仅位于右侧边缘，未见棋子。基线仍在部分帧产生圆候选，说明仅圆形检测会出现误检；本轮不把这些候选当作棋子识别准确率证据。
需后续验收：棋盘/棋子视野与距离、光照与深度完整性、倾斜和遮挡下检出率、误检率、实物定位误差、具体棋子分类、手眼与 TCP 标定、运动中时间同步、自动夹取闭环。
无帧失效逻辑已实现；物理拔线与重新插入场景未做实测，持续硬件故障需检查后重启节点。

## 7 来源

- [项目规格](https://app.notion.com/p/3ff9f9a4e09e81fca280e1f3e4c42140)
- [SDK 固定版本](https://github.com/realsenseai/librealsense/tree/v2.58.1)
- [官方 Jetson 安装说明](https://github.com/realsenseai/librealsense/blob/v2.58.1/doc/installation_jetson.md)
