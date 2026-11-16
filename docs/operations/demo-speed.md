# 排一行演示与设备恢复

以下入口从 Jetson 项目根目录执行，开发构建见根 README。
源码归属为 chess_robot_tasks；入口仍使用现有 chess-robot-arm-contact-2mm-20261105.service，其安装和存档没有随目录整理切换。

## 1 运行入口

```bash
cd /home/kuang/workspace/chess_robot
bash src/chess_robot_tasks/scripts/run_speed_demo.sh row --workspace-ready
```

| 操作参数 | 实际行为 |
| --- | --- |
| prepare（兼容名 recover） | 拉起当前机械臂服务，恢复参考，按需初始化/张爪并回 Home |
| row --workspace-ready | 先准备，再扫描、选择棋子、抓放及复核 |
| scan | 先准备，再执行现场扫描配置 |
| vertical | 准备后执行空爪竖直往返，会运动 |
| activate | 确认停稳存档后重启同一机械臂服务，再恢复参考和读回参数 |
| shutdown | 空爪/Home/停稳存档确认后停止服务；看到 POWER_OFF_READY 再断电 |

恢复时只有确认停机后没有手动拖动、碰撞等未观测运动，才附加 --no-unobserved-motion。
--workspace-ready 表示桌面已准备、夹爪无物体、手已移开；目标行允许已有棋子，由占位策略处理。
--workspace-clear 是目标行为空时的原有确认方式。历史脚本中的授权文字不代表新一轮实体授权。

准备可能使能、初始化夹爪、张爪和移动到 Home，不是只读检查。
参考恢复失败时不进入后续步骤；不自动清零历史、不回退旧存档，不重试结果不确定的设备命令。
中断和故障处理沿用源码现行流程；取消回执不能单独证明实体停稳。

## 2 配置和当前行为

| 内容 | 配置入口 |
| --- | --- |
| 扫描、空中移动、Home 与接触段速度 | config/arm/demo-speed.yaml |
| 槽位、高度、力度、观察、近邻和占位策略 | config/pick/demo.row.live.json |
| 日常 Home / 独立历史基准 | config/arm/home.yaml / config/arm/twist_reference.yaml |
| 扫描点和资格 | 所选 JSON 的 scan_config；现场演示仍引用 config/scan/scan.site.json |

1. **占位**：当前 row_occupied_policy=occupant_first。当前槽位占位者优先；标准位置且独立复核通过的棋子可计入完成。多重占位按现行流程选择后续空槽，不能用原来的 skip 说明代替它。
2. **观察**：不明确时按 active_observation 的方向、偏移和高度范围补查；缺证据与明确占位分别处理。具体数值以所选 JSON 为准，不在文档另维护一套默认值。
3. **近邻和朝向**：pick_neighbor_policy=finger_model 使用完整指体包络；空爪时按 pick_yaw_rechecks_deg 尝试配置朝向。最终下降重新核对当前测量和规划。
4. **复核**：row_verification、pick_target_observation 和 target_recheck_attempts 分别定义对应窗口和次数。保留未完成和待确认结果，不以程序退出代替成功落位。
5. **生效位置**：参数修改后核对实际进程读取值。扫描配置的 guard_file 与运行节点参数严格比较，部署未迁移前不能只改名守卫文件。

调度实现为 src/chess_robot_tasks/chess_robot_tasks 下的 demo.py、ros_io.py 和 startup.py。
现行门限、等待与恢复条件以所选配置和上述源码为准。

## 3 记录与排错

准备记录为 runs/demo_startup_*/startup-result.json，摆放记录为 runs/row_speed_*/run-result.json。
现场绕转存档仍为 .runtime/arm-checkpoint.save，审计仍由现有驱动维护，不复制另一份存档续写。
失败时从本轮阶段、原始原因和相关节点日志追查；“视觉不可用”“恢复失败”本身不是根因。

工作区与现场冻结视觉版本不同，开发构建检查不能代替整机验收。
原有操作演变和各次实体结果保存在 [历史记录](../../runs/records/operations/demo-speed.md)。
