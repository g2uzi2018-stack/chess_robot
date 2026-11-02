# 验证结果与证据索引

## 1 当前结论与边界

1. **1秒反馈安装**
   - [2026-10-31变更与加载](feedback-window-1s-20261031.md)：147单元测试（104 arm / 23 checkpoint / 20 twist）、3个loopback场景通过。
   - 650ms空窗保留参考；1200ms空窗或重复旧帧失效；只停稳重启加载，无新的真实运动资格验收。
   - 运行前缀 `runs/feedback_window_1s_20261031_01/install`；PID2798832、SHA256见报告，不能按旧PID替换当前实例。
2. **停稳存档与审查**
   - [机制、迁移与审查修复](checkpoint-recovery-20261031.md)：135→142项为当时分阶段结果，当前合计见1秒报告。
   - 原位恢复保留扭转；已测空闲进程重启与历史断电边界，部署后尚未追加真实物理断电试验。
   - 主要证据：`runs/checkpoint_recovery_20261031_01/`、`runs/checkpoint_review_20261031_01/`。
3. **标定与激光粗检**
   - [48观测、留出一致性与实体结果](handeye-20261030.md)：36训练来自24位姿，12留出来自4位姿。
   - 留出中位0.208875mm、最大0.339257mm/0.111317°仅是内部一致性，候选未发布生产TF。
   - CH65→CH56→CH32→CH65粗检由用户确认可接受，未量化实体毫米误差；最后保持CH65。
   - 证据：`runs/handeye_campaign_20261030_01/`、`runs/corner_validation_20261031_02/`、`runs/corner_validation_20261031_03/`。

## 2 历史记录怎样使用

1. **保留原验证上下文**
   - 100ms失败与200ms续测分别保留；不能把旧运动结果写成1秒窗口资格。
   - 旧报告中的“当前/待部署/失败”均指报告冻结时的版本；现用入口见[日常操作](../operations/arm-quickstart.md)。
   - [历史恢复契约与三阶段结果](reference-recovery-history.md)合并被替代的实现阶段叙述，保留证据、SHA和失败修复事实。
2. **证据保存边界**
   - 本目录小型JSON、候选YAML、PNG与验收摘要保留；原始audit/图像/CDR和部署在被Git忽略的 `runs/`。
   - 历史报告不代表全工作区、安全认证、物理精度或当前版本实机验收；模拟与真实结果分开。
   - 三份恢复实现报告的原文在 `runs/docs_cleanup_20261031_01/originals/`，删除/合并映射与SHA在同目录manifest；不把备份当第二套活跃文档。

## 3 保留的分项验收

1. **历史证据目录**
   - [C++ 连接与真实反馈验证](actual-feedback-2026-10-24.md)。
   - [AG95 迁移与装配检查（2026-10-23）](ag95-assembly-2026-10-23.md)。
   - [AG95 第一轮几何修正（2026-10-24）](ag95-refinement-2026-10-24.md)。
   - [验收记录](arm-node-2026-10-24.md)。
   - [业务节点验证与 ServoJ 时序排查](business-node-20261025/REPORT.md)。
   - [相机与位姿职责拆分](camera-io-20261025/REPORT.md)。
   - [更换用户提供的新相机/支架 STL（2026-10-24）](camera-v2-2026-10-24.md)。
   - [C++ connection baseline — 2026-10-24](connection-baseline-2026-10-24.md)。
   - [D405 1280×720 启动失败排查与 848×480 验证（2026-10-29）](d405-1280-20261029.md)。
   - [D405重新插拔后1280复测 — 2026-10-30](d405-replug-20261030.md)。
   - [反馈空窗独立诊断（2026-10-26，0d5a2c9基线）](feedback-gap-20261026/REPORT.md)。
   - [Independent final acceptance: new-board handeye campaign](handeye-20261030/independent-final-review.md)。
   - [Portability release verification — 2026-10-30](handeye-20261030/release-verification.md)。
   - [Hand-eye orchestrator implementation acceptance (2026-10-29)](handeye-node-20261029.md)。
   - [机械零位记录与工作 Home 候选](home-reference-2026-10-24.md)。
   - [Home → 实际 TCP → Home：200 ms 续测验收](home-roundtrip-200ms-20261026/REPORT.md)。
   - [Home→TCP 目标→Home：工具就绪，实机范围探索阻塞](home-roundtrip-20261025/REPORT.md)。
   - [Home 到点往返工具验收](home-roundtrip-20261025/TOOL-REPORT.md)。
   - [Home → 实际 TCP → Home 范围验收：2026-10-26](home-roundtrip-20261026/REPORT.md)。
   - [Nova5 接口、模型与运动执行验收：第一阶段证据报告](nova5-2026-10-20.md)。
   - [Nova5 RViz 模型与模拟运动检查](nova5-rviz-2026-10-20.md)。
   - [SHA-bound disabled cold-restart recovery — 2026-10-30](power-fault-recovery-20261030.md)。
   - [预览节点 30 Hz 更新](preview-30hz-20261025.md)。
   - [局域网预览节点交付记录](preview-node-20261025.md)。
   - [预览职责独立与无数据占位验证](preview-owned-20261025/REPORT.md)。
   - [当前姿态参考图与一次小范围实机验证](small-motion-2026-10-24.md)。
   - [chess_robot 软件修复与离线验收](software-repair-20261025.md)。
   - [中国象棋夹取视觉节点第一版](vision-node-20261025.md)。
   - [视觉与预览机器可读证据](vision-preview-20261025/README.md)。

- [2026-10-31 象棋识别与曝光联调](xiangqi-recognition-20261031.md)：只读分割、表面XYZ及网页试验；正式C++位姿节点与自动夹取尚未验收。

- [2026-10-31 Home与棋子跟随编号](xiangqi-tracking-20261031.md)：一次受检Home、位置关联编号、独立C++预览及局域网实测；姿态和类别仍有实验限制。

- [2026-10-31 识别GPU核验与性能优化](xiangqi-gpu-performance-20261031.md)：CUDA设备证据、局部CPU处理与独立推理进程、离线/实时耗时边界。

- [2026-10-31 保持FP32的并行流水线](xiangqi-parallel-20261031.md)：共享内存与GPU/CPU重叠、同帧一致性、7.4→15.6Hz、断电反馈边界。
