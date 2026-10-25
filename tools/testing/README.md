# ROS 端到端回归（仅回环夹具）

`arm_controller_fixture.py` 只监听 `127.0.0.1` 的随机端口，使用项目 URDF 做 FK/数值 IK，生成的数据明确标记为模拟。测试节点使用 `test_fixture=true`，话题为 `/simulation/arm/*`，不连接真实控制器。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 python3 tools/testing/arm_ros_integration.py \
  --out runs/new_ros_loopback
```

输出目录必须不存在。需要 rclpy、numpy、scipy、PyYAML。覆盖只读无运动、任务排他、取消、反馈失效、精确抓取确认、超时不撤离、运动回包异常分类和 Stop 故障锁定。真实工作区扫描与实机调试另见 `tools/analysis/business_node_acceptance.py`。
