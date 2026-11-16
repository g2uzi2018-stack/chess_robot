# 中国象棋夹取

Nova5 机械臂、AG95 夹爪和 D405 相机组成的 ROS 2 Humble 工程，提供设备控制、棋子识别、桌面扫描和抓放调度。

## 1 环境与开发构建

开发基线位于 Jetson AGX Orin：`/home/kuang/workspace/chess_robot`，Ubuntu 22.04 / aarch64。
需要 ROS 2 Humble、C++17、CMake、colcon，以及各包 package.xml/CMakeLists.txt 声明的依赖。
相机使用本机 librealsense 2.58.1；OpenCV、Eigen、yaml-cpp、FCL、Boost 和 GStreamer 由本机开发环境提供。
Python 感知另外需要原训练工程的 PyTorch、Ultralytics、TensorRT 环境和模型文件；不随本仓库分发。

从项目根目录执行：

```bash
cd /home/kuang/workspace/chess_robot
bash tools/build.sh --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo   -Drealsense2_DIR=/home/kuang/.local/realsense-2.58.1/lib/cmake/realsense2
source tools/env.sh
```

默认构建在 `build/ros`，安装在 `install/dev`。独立维护工具使用 `bash tools/arm/build.sh`。
完整目录说明在 [docs/build-layout.txt](docs/build-layout.txt)，构建时恢复 `build/README.txt`。
只改一部分时使用 `bash tools/build.sh --packages-up-to <包名>`，配置选项同上按需传入。

## 2 使用与现场版本

| 需求 | 入口 |
| --- | --- |
| 机械臂、Home、存档恢复 | [机械臂操作](docs/operations/arm-quickstart.md) |
| 控制台与夹爪 | [网页操作](docs/operations/web-panels.md)；本机现场端口为图像 8088、控制台 8089 |
| 扫描和抓放 | [任务包](src/chess_robot_tasks/README.md)、[抓放操作](docs/operations/pick-place.md) |
| 演示、正常停机与恢复 | [演示操作](docs/operations/demo-speed.md) |
| 感知与场景节点 | [感知包](src/chess_robot_perception/README.md) |
| 工具与离线分析 | [tools/README.md](tools/README.md) |

纯规划扫描的最短入口：

```bash
source tools/env.sh
ros2 run chess_robot_tasks scan_controller   --config config/scan/scan.current.json --output runs/scan_NEW
```

输出目录换成尚不存在的批次名。规划需要相应 ROS 节点和当前配置；该命令没有 `--execute`，不会执行扫描运动。
实际运动的前提、完成条件、停止和恢复方式见对应操作文档。Ctrl+C 的取消请求不能单独证明实体已停稳。

**开发工作区与现场正在运行的版本不同。** 现场服务仍使用原来的冻结目录和安装；修改、构建此工作区不会自动切换服务。
当前开发构建不包含那些冻结目录中独有的视觉或预览修改。切换前须分别核对源码、参数、模型、状态及实际服务入口。

## 3 目录与修改入口

| 目录 | 内容 |
| --- | --- |
| `src/` | ROS 包、业务实现和共用库；驱动、感知、任务、接口、描述资源各有归属 |
| `tools/` | 维护、标定辅助、分析、训练和既有验证工具，统一说明入口 |
| `config/` | 工作区选择的扫描、抓放和机械臂参数；包内 config 按节点需求保存直接相关的参数与配套配置 |
| `data/` | 已采用的长期输入；手眼文件及来源在 `data/handeye/` |
| `state/` | 新增跨运行状态的归属，按实际需要创建；不复制一份现场存档另行续写 |
| `runs/` | 按批次保存的资料；`runs/records/` 保存原有历史记录 |
| `log/` | 运行日志；`source tools/env.sh` 设置日期目录和格式，实际文件命名见 [日志说明](docs/operations/logging.md) |
| `build/`、`install/` | 开发构建和安装输出；不会替代已经运行的进程 |
| `docs/` | 当前接口、配置、操作、模型和工具说明 |
| `reference/` | 厂家原件、来源和许可证 |

现场 `.runtime/` 仍由现有部署使用，包含有效绕转存档和审计数据；活动的历史 runs 也保留。
它们尚未做部署迁移，不能直接作为缓存清空。配置读取和覆盖位置见各模块说明。

## 4 验证与边界

C++ 既有检查可按 [构建说明](docs/build-layout.txt) 指定的构建区执行 CTest/colcon test；视觉纯函数检查保存在感知包 test/。
视觉既有检查 test_passive_audit_epoch_duplicate_rotation_and_motion 在原工作区也失败：计时重置后的第一帧仍与旧计时做差，代码拒绝该帧，而测试预期通过；现有代码与检查预期不一致。
不运行冒烟测试。构建、离线检查、现场识别和实体抓放是不同验证范围。

真实 TCP/夹指接触几何、标定绝对精度及全工作空间能力仍受原有验证范围限制；识别成功不等于抓取可执行。
既有实验结论和原始资料见 [历史记录](runs/records/validation/README.md)，它们描述当时版本，不代表当前构建已实机验收。

开发前读 [AGENTS.md](AGENTS.md)。现行文档从 [docs/README.md](docs/README.md) 进入，节点调整必须同步 [ROS 接口](docs/ros_interfaces.md)。
