# 业务机械臂验证与工作区探索

运动仅经 `/arm/move` → Planner → 厂家 FK/IK → 扭转/几何保护 → Executor → ServoJ。默认只读，保存的使能和正式认证标志均为 false。维护 MovJ 不参与业务验证；到 Home 不自动清零参考。

当前软件基线见 [2026-10-25 修复报告](../validation/software-repair-20261025.md)。
用户提供固件 4.6.2.4，采用固定 C++ V4；`motion.paused=true`，本轮不恢复实机。

## 执行与约束

同一命令连接有界异步收发：应用每 30 ms 发点，FIFO 核对回包原命令及递增队列 ID；探索配置最多 6 条未确认，单条确认期限仍为 150 ms，窗口容量覆盖该期限内的 30 ms 发送点数。确认超时仍立即判执行失败并禁止发点；仅接收超时时，保留原独占连接，最多额外 500 ms 仅接收已发送 FIFO，再用该连接 Stop，计入原有 4 s 停止预算。排空失败或格式损坏时也保留原连接，禁止新运动/重连，只尝试一次同连接 Stop；在有界时间内只接受 Stop 自己的完整回包。物理断链或缺少 Stop 回包仍明确失败并锁定。Stop 匹配自己的回包，并由新鲜、低速、mode=5 反馈持续确认停止。执行故障立即终止本批；Stop 失败锁定 FAULT。旧轨迹文件仅作证据，不是运动输入。

`servo.command_time_s` 配置控制器跟随时间，与发送周期、确认期限分别约束跟随、发送和回包。探索当前默认跟随时间等于 30 ms 发送周期，aheadtime=100、gain=200，默认 low 档；具体批次参数及实测结果以报告和源快照为准。TCP_NODELAY 和逐次接收后的 Linux QUICKACK 请求减少小包延迟，QUICKACK 是临时请求。

移除 0.02 rad/s / 0.05 rad/s² 的调试硬上限及 0.032 rad/s 派生停机判据。low=0.708 rad/s、1.416 rad/s²；high=2.36 rad/s、4.72 rad/s²，默认档和两档参数均可配置，来自固定官方 Nova5 ROS2 配置。当前设备的再现/缩减上限仍需现场核对；这些值不是本机物理速度认证。ServoJ 不注入速度百分比命令，速度来自轨迹时间律。跟踪保护统一采用已有正常配置 0.15 rad，低速旧模板中的 0.005 rad 门限不再沿用；停稳速度和 dwell 不提高。C1 插值与单一五次时间律后逐帧复核速度、加速度、竖直偏差及全部几何/扭转约束。

几何筛选保留：桌面 z=-10 mm、活动部件留 20 mm，非相邻包络间距 10 mm、关节限位余量 5°、归一化 Jacobian 最小奇异值 0.05、条件数 80。软件净/总扭转上限各 2π，阶段子限更小。TCP 是标称 URDF 变换，未独立标定；附件、安装或环境变化需重新核实模型。

## 探索边界

`backend=exploration` 使用 `config/exploration.yaml` 的有序 profile/边界，替代固定 4 cm/0.12 rad 的临时包络。每个 Goal 按 profile 检查相应 TCP/关节半径和扭转子限；完整路径及反馈仍受硬保护。

| profile | TCP 距 Home / m | 关节距 Home / rad | 净/总扭转子限 |
|---|---:|---:|---:|
| low / high / near | 0.08 | 0.3 | 30° |
| inner | 0.18 | 0.7 | 60° |
| middle | 0.40 | 1.5 | 120° |
| outer | 0.70 | 3.0 | 180° |
| full | 1.10 | 6.0 | 360° |

这些是试验边界，不代表整个体积均已验证。旧 `commissioning` 模板保留原小范围约束；探索模式不报告正式认证。

`business_workspace_explorer.py` 生成逐级预备点、3×3×3 网格、两个高度的四向边界探测（加密到 25 mm）、左右跨区往返和 ±90° 分步朝向清单。采用 50 mm 预抓高度，分别真实执行预抓、下潜、撤离，不伪造夹爪抓取确认，也不逐点回 Home。

合理的规划拒绝保存原因并继续其他点，必要时另行规划并反馈确认高位桥接路径。规划成功与反馈到位分别记账。执行故障停止并排查，修复验证后使用新目录、新目标规划继续新批，禁止重放失败轨迹。

## 参考与跨进程衔接

首次已知线缆中性 Home 可显式调用 `/arm/initialize_reference`。中途或跨进程继续时使用 `/arm/restore_reference`，提交时间有序的原始 `audit/feedback.bin` 文件，并明确确认记录空窗内没有移动、其他运动程序或控制器复位。

恢复从历史中的真实 Home 重放完整相位，不在当前位置初始化。须使用匹配的 URDF/Home/TCP 来源。格式缺失、时间倒退、文件内部反馈中断、关节/扭转越界、非静止端点、当前反馈陈旧、末端关节差超过 0.001 rad、已有有效参考或锁定故障，都会拒绝恢复。声明不能替代未知历史。若旧记录停在 mode=7 的 ServoJ 保持状态，必须先记录足够的静止 dwell（当前 0.4 s、速度不超过 0.002 rad/s、关节跨度不超过 0.001 rad），再追加真实 Stop 后的 mode=5 反馈并核对衔接；单独的 mode=7 终态始终拒绝恢复。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1
# 输出目录必须新建；默认只规划。历史须与实机和配置匹配。
python3 tools/analysis/business_workspace_explorer.py --controller-ip 192.168.30.234 \
  --out runs/new_workspace_plan --history-audit /absolute/trusted/audit/feedback.bin \
  --attest-no-unobserved-motion
```

本次明确授权的实机批次另加 `--execute`。修复后可用 `--completed-plan` 指向前批结果，提交完整历史链；原始 Home 来源可核验时可加 `--skip-recovery-home`，从恢复的当前姿态继续。`--validate-speed` 先执行新的上/左/右路径及可测位移后的取消停止，并复核原始速度，再继续清单。`--enable-disabled` 仅在无错误的未使能静止状态请求使能，不清错、不切模式。

选解仍为总扭转 → 净扭转绝对值 → 耗时，采用 5° 中性带、2° 角度滞回和 1 s 耗时滞回；只在当前/Home 两个厂家 seed 的受检候选内比较，不证明全逆解全局最优。

最新状态、源版本、原始证据与三维图见[验证报告](../validation/business-node-20261025/REPORT.md)。
