# 停稳两秒观测、锁定与单次消费

实时检测独立运行；相机图像无需机械臂 reference_valid 或 mode5。旧 `frame_id=base_link` 与 `valid=基座几何有效` 保持原义，相机坐标不冒充基座坐标。所有 motion_ready/grasp_ready 仍为 false。本模块只有视觉数据接口，不控制机械臂。

## 默认流程

启动后自动等待可信停稳：最近实测反馈不超过 0.2 秒，关节速度及相邻关节变化率不超过 0.002 rad/s，停稳至少 0.6 秒，静止区间关节漂移不超过 0.001 rad。同一份反馈可以覆盖多个图像；反馈 stamp 原样保留。当前旧 arm 节点没有暴露控制器序列号，视觉组件被动读取它已有的旋转反馈审计，检测已观察到的控制器时间戳倒退、长缺口和运动；不会建立控制器连接。审计不可读或过时将暂停资格，图像仍可运行。epoch 是本进程观察到的连续区间 token，不是厂商 boot ID。

随后异步采样至少 2 秒、20 个不同 RGB-D 帧对；每目标有效比例至少 80%，单目标及全局观测缺口不超过 0.3 秒。启动窗口须看到一帧所有当前检出目标都具有完整有效中心，之后允许少量目标几何缺帧，各目标分别计算有效比例。采样期间采用完整画面，保留原深度、平面及裁剪边缘筛选；目标集合无法装入 720 方框时不会强行裁剪。

使用跨帧关联后的目标，输出相机/基座表面中心中位数、MAD、径向 P95，类别加权投票和法向 RMS。中心锁定要求径向 P95 默认不超过 3mm，是相对于现有 18mm 起始关联门限更严格的重复性筛选。法向 RMS 沿用已有 1.5° 稳定性标记阈值：超过时输出 temporally_stable=false，不阻止通过原深度/平面质量与中心重复性筛选的表面中心锁定；法向与抓取资格分别表达，orientation_valid_for_motion 始终 false。这些值不是绝对精度或抓取姿态验收。配置可通过 XIANGQI_OBSERVATION_POLICY 指向 Policy 字段的 JSON；默认值写入每个快照。

锁定结果含 snapshot_id、原始采样起止和锁定时间、关节中位数、观测 epoch、模型/外参/URDF 哈希及策略哈希。查询不会重采样或改时间戳。FrameInfo 的彩色/深度帧编号也参与去重；主机 SDK 取帧 stamp 不是曝光时间。

## 视觉命名空间接口

请求 String JSON：`/pose/xiangqi/observation/request`。应答：`.../response`，通过 request_id 配对。状态与最后锁定快照分别为 `.../status`、`.../snapshot`，均 transient_local、depth=1；已锁定快照数据不可变，当前资格只看状态。

```json
{"action":"collect","request_id":"r1"}
{"action":"query","request_id":"r2"}
{"action":"consume","request_id":"r3","snapshot_id":"实际ID","operation_id":"业务操作唯一ID"}
```

collect 是非阻塞重采样请求；等待 status=locked 后 query。查询可带 snapshot_id；省略时返回当前保存的快照。consume 必须匹配当前、未过期且已复核场景的快照。同 operation_id 幂等返回不可变数据，另一个 operation_id 无法重复使用该快照；操作账本最多 1024 条，满时拒绝新消费并要求归档/重启视觉会话。账本副本及快照保存在运行目录 snapshot-audit.jsonl。重启不恢复旧结果的当前资格。

```bash
source /opt/ros/humble/setup.bash
source /home/kuang/workspace/chess_robot/install/setup.bash
export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1
python3 /home/kuang/workspace/chess_robot/tools/testing/xiangqi_tracking/observation_client.py collect
python3 /home/kuang/workspace/chess_robot/tools/testing/xiangqi_tracking/observation_client.py query
python3 /home/kuang/workspace/chess_robot/tools/testing/xiangqi_tracking/observation_client.py consume --snapshot-id 实际ID --operation-id 唯一操作ID
```

consume 只记录数据被某操作领取，不表示已允许运动，更不会发动作。动作执行层仍须保留原安全门限。一次采集默认供一个操作，后续操作显式 collect；观测到动作后会重新等待停稳并采样。

## 保留、复核和失效

短时基座资格失败会挂起 tracker，绝不向基座 tracker 注入相机坐标。短时失联保留历史 ID/快照，暂停当前资格；反馈恢复且姿态一致后重新停稳可恢复。发生真实运动、参考/策略身份或 epoch 不连续、长于 2 秒的未观测、相机帧计数复位时退役或重新关联。近似同价匹配仍标记 identity_ambiguous，不强行继承身份。

锁定后继续用相机表面中心和类别做轻量复核。目标数量变化、位置变化超过 6mm 或关联歧义，首帧暂停消费，连续三帧撤销并重新采样；高置信类别变化也触发复核。最新场景证据须在 0.5 秒内。默认快照最长保留 60 秒当前资格，适合两秒后操作的流程；历史数据继续保留。该复核不能保证检测器未检出的目标或小于门限的变化被识别，使用方不能跳过实际运动层的再确认。

状态：collecting（采集中）、locked（已锁定）、pending_confirmation（待确认）、needs_resample（需重采样）、consumed（已领取）。预览明确区分实时值与 LOCKED 历史值，当前资格在状态中单列。源缺失/晚启动时等待并自动恢复。回调只缓存数据和请求；推理与状态机在主循环运行，两秒采样不阻塞 ROS 回调。
