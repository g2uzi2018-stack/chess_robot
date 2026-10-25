# 业务机械臂调试

运动入口为 `/arm/move` → Planner → 厂家 FK/IK → 扭转/几何检查 → Executor → ServoJ。默认只读；维护工具 MovJ 不参与业务验证。

`config/commissioning.yaml` 保存 `hardware.enable=false`，正式验证标志仍为 false。调试模式要求真实几何保护，规划速度 ≤0.02 rad/s、加速度 ≤0.05 rad/s²；测试 profile 0.01 rad/s、0.04 rad/s²。反馈速度监测阈值 0.032 rad/s 是软件故障门限，不是控制器硬限速。扭转净值/总量各限 20°。

选解先比较总扭转，再比较净扭转绝对值，最后比较耗时。默认中性带 5°、角度换支滞回 2°、耗时滞回 1 s。中性带和滞回仅影响已通过整条路径保护的候选解，不放宽保护。旧 cost 参数为兼容而接受，不再参与排序。

启动前必须有已知的线缆中性 Home、新鲜静止反馈、控制器 mode=5 和 TCP 控制权。脚本要求显式 `--initialize-reference-at-home`，每次启动仅建立一次参考；到 Home、取消或故障后均不会自动重置。未知绕转历史必须先现场核实，不能用初始化抹去。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1
# 输出目录必须不存在。以下不产生运动。
python3 tools/analysis/business_node_acceptance.py --controller-ip 192.168.30.234 \
  --initialize-reference-at-home --expanded-plan-only --out runs/new_local_plan
```

`--scan` 扫描默认 250 mm 工序，只规划。`--execute-local --expanded-local` 是本次已授权调试的 25/30/35 mm 分批脚本，采用 20 mm 预抓高度，在 40 mm TCP / 0.12 rad 关节候选包络内选择已通过点位。每个新点先完整工序预检，再分别执行 PREGRASP、DESCEND、RETREAT 和 Home；不伪造夹爪确认。部分位于球形包络边缘的点只验证 PREGRASP/Home。**本轮在 25 mm 批回 Home 时发生调度保护，30/35 mm 没有执行；默认模板仍保持 30 mm / 0.1 rad。**

任一执行或本批预检失败即停止，不重放，不自动回 Home。取消必须由 Stop 成功回包与新鲜、低速、mode=5 反馈共同确认。Stop 失败锁定 FAULT；保存原始证据并排查后另行决定恢复，不能通过换节点或初始化绕过。脚本结束会关闭本次软件参考；当前实现没有跨进程参考恢复功能，停在非 Home 后不可重新初始化来跳过此条件。

TCP 为标称 URDF 变换，未独立标定。几何筛选保留桌面 z=-10 mm、活动部件 20 mm 余量、非相邻包络 10 mm、关节限位 5°、归一化 Jacobian 最小奇异值 0.05、条件数 80；现场额外障碍不在模型中。新附件/安装变化需重新核实几何和 TCP。

最新结果和三维图：[验证报告](../validation/business-node-20261025/REPORT.md)。规划通过、实机到位、规划拒绝和实机失败分开记录；坐标包围盒不代表连续已认证工作区。
