# ROS 端到端回归（仅回环夹具）

`arm_controller_fixture.py` 只监听 `127.0.0.1` 的随机端口，使用项目 URDF 做 FK/数值 IK，生成的数据明确标记为模拟。测试节点使用 `test_fixture=true`，话题为 `/simulation/arm/*`，不连接真实控制器。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 python3 tools/testing/arm_ros_integration.py \
  --out runs/new_ros_loopback
```

输出目录必须不存在。需要 rclpy、numpy、scipy、PyYAML。覆盖只读无运动、任务排他、取消、反馈失效、精确抓取确认、超时不撤离、运动回包异常分类和 Stop 故障锁定。真实工作区扫描与实机调试另见 `tools/analysis/business_node_acceptance.py`。

## 历史恢复与视觉解耦回归

`run_history_vision_regression.sh BEFORE_SOURCE_TGZ [OUTPUT_DIR]`会在新的隔离输出目录构建，并运行localhost的历史生命周期、视觉资格与叠加测试。它不启动实体控制节点或相机，但仍依赖指定旧源码归档、ROS安装和外部Python运行器；只克隆Git仓库并不足以获得所有基线与模型依赖。

`xiangqi_tracking/test_history_loopback.py`使用127.0.0.1假控制器、固定隔离domain118和`/simulation/arm/*`，覆盖纯恢复、使能异常后的确认、坏档/身份/端点拒绝及进程重启。运行前先检查脚本使用的构建路径、环境与测试夹具，不要把现场service当成夹具启动。

面板和夹爪的独立集成脚本不一定注册到CMake/CTest；`colcon test`通过不自动证明这些脚本已运行。各工作包应显式记录实际执行入口、源码版本和结果。
