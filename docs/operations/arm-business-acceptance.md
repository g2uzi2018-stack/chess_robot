# 业务节点实测与可达性检查

业务运动使用 `/arm/move`，经同一个 Planner、SDK、扭转跟踪器和 Executor；独立 `tools/arm` 维护工具不参与目标执行。

默认 `read_only`。启动入口 `arm.launch.py` 会加载配套几何保护：桌面 z=-10 mm，活动部件保留 20 mm，另检查非相邻包络和雅可比。模型或附件变化后须重新导出几何文件。

`config/commissioning.yaml` 为 Home 周围小范围调试覆盖配置，保存的 `hardware.enable` 仍为 false。它不会把正式运行的验证开关标成 true，状态 `hardware_certified` 也仍为 false。

模板约束：TCP 距 Home ≤3 cm、各关节距 Home ≤0.1 rad；规划速度上限 0.02 rad/s、加速度 0.05 rad/s²；实际测试 profile 为 0.01 rad/s、0.04 rad/s²；净/总扭转各 20°。反馈速度监测阈值为 `1.5 × 规划关节上限 + 0.002 rad/s`，模板为 0.032 rad/s。达到 Home 不会自动重置扭转参考。

实测脚本显式加 `--execute-local` 才能运动，扫描和负向测试只规划。它需要启动时在已知、线缆中性的 Home 附近，通过新鲜静止反馈后显式建立本次软件参考。任一非预期失败即终止后续运动，不重试、不用 MovJ 绕过。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1
# 指定一个尚不存在的输出目录；以下只读。
python3 tools/analysis/business_node_acceptance.py --controller-ip 192.168.30.234 \
  --scan --out runs/new_business_scan
```

真实运动使用同一脚本的 `--execute-local`，仅限已授权的现场调试。取消后必须收到 Stop 成功回包，并由新鲜反馈确认静止；失败会锁定 FAULT，后续 Home 不能清除此故障。

Stop 的命令会话、回包预算和停止确认分别有日志；回包不是机械停止的证明。结果中的 TCP 来自编码器与标称 URDF，并非外部测量精度。
