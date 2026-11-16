# 扫描与抓放任务

本包通过公开 ROS 接口编排扫描、抓取、排一行、叠放和设备准备，不实现设备通信或视觉算法。
`scan_controller.py` 负责扫描；`demo.py` 负责流程；`ros_io.py` 连接 arm、gripper、scene；`startup.py` 负责既有准备流程。

从项目根目录构建：`bash tools/build.sh --packages-up-to chess_robot_tasks`，然后 `source tools/env.sh`。
纯规划扫描：`ros2 run chess_robot_tasks scan_controller --config config/scan/scan.current.json --output runs/<新批次>`。
抓放入口：`bash src/chess_robot_tasks/scripts/run_once.sh --help`。
现场演示入口：`bash src/chess_robot_tasks/scripts/run_speed_demo.sh`，无参数只打印用法。

项目配置在 `config/scan/`、`config/pick/`；输出在显式指定的 `runs/<批次>/`。
完整操作和现有动作条件见 [抓放操作](../../docs/operations/pick-place.md)、[演示与恢复](../../docs/operations/demo-speed.md)。
接口见 [ROS 接口](../../docs/ros_interfaces.md)。构建此包不启动或切换现场服务。

扫描、抓放与设备准备按阶段记录 ROS 日志，复用已有 task_id；大数组保留在批次资料中。阶段输出不表示动作已经完成，详见 [日志说明](../../docs/operations/logging.md)。
