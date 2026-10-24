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
