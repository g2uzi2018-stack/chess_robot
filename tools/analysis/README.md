# 工作区和奇异性离线分析

不连接控制器、不发布实机状态或运动指令。Python 做 URDF/网格处理、数据编排和标准静态绘图；
Eigen/FCL C++ 内核批量计算 FK、雅可比、地面包络及非相邻组保守距离。
`workspace_policy_probe.cpp` 额外调用当前已构建的真实 Planner，测试其能否接收已知 FK 验证的奇异终态。

环境：C++17、Eigen3、yaml-cpp、FCL、pkg-config；Python 依赖在 `requirements.txt`。
AGX 已有 `runs/urdf_tools_env`。Planner 审计需先构建 ROS 2 业务包，主几何计算本身不依赖 ROS。

脚本第一个可选参数是输出目录；额外参数传给几何分析器。因此带参数时应显式给输出目录：

```bash
cd /home/kuang/workspace/chess_robot
bash tools/analysis/run_workspace_study.sh runs/workspace_review \
  --ground-z -0.01 --clearance 0.02 --descent-speed 0.01
```

要用默认参数与输出目录，直接运行：

```bash
bash tools/analysis/run_workspace_study.sh
```

输出包含图、CSV、JSON、二进制采样配置/指标及网格数据 NPZ。不要将生成的大型采样文件提交到 Git。
`docs/analysis/arm-workspace-2026-10-24.md` 保存本次结果与边界。

默认假设：桌面 z=-10 mm；活动部件地面余量 20 mm；静止底座免除地面余量；
非相邻包络间距 10 mm（固定末端组及相邻组明确排除）；21 个夹爪开度样本；
25 mm 笛卡尔网格、8 个典型逆解分支、固定 Home 夹爪朝向；
归一化雅可比最小奇异值 ≥0.05、条件数 ≤80、关节限位余量 ≥5°，10 mm/s 竖直方向速度参考；
同分支上方 250 mm 通道筛选。阈值是本次工程筛选参数，不是厂家认证值。

结果是离线候选范围，不能替代实机 TCP/安装误差标定、业务节点的连续状态和轨迹验证。

## 业务节点证据工具

`business_node_acceptance.py` 为真实 Action 的只读扫描/显式实机验证入口，使用前阅读
`docs/operations/arm-business-acceptance.md`。`--initialize-reference-at-home` 必须显式给出；
失败终止本批，不自动重放。没有跨进程参考恢复功能。

`business_evidence_audit.py RUN --evaluator PATH` 使用 `workspace_eval.cpp` 的离线二进制
复核精确规划轨迹及真实反馈的桌面、非相邻包络、限位、奇异性余量。它不连接控制器，
输入目录须有 `results.json`、压缩轨迹和 `joint_feedback.csv`。

`business_evidence_plot.py --workspace OLD_SCAN --local-planning LOCAL_PLAN --physical RUN --out OUT`
生成三维 PNG 与交互 HTML，严格区分历史全局规划、本轮局部规划、实机到位、停止和失败。
本次全局背景版本为 0b15c6d；图中的空白不是通过区域。HTML 的 Plotly 库来自固定 CDN。

`business_latency_probe.py --controller-ip IP --out NEW_DIR` 仅发送只读 GetAngle，记录 TCP_INFO、
真实反馈及主机定时抖动；不初始化参考，不使能，不发运动或 Stop。仅在运动所有者退出后运行，
避免与业务节点争用控制器的唯一命令连接。事后只读延迟不代表运动期间 ServoJ 的时序保证。
