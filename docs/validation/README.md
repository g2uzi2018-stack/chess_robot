# 验证结果与证据索引

## 1 当前结论与边界

[2026-11-04单颗红卒抓取与20mm抬升](xiangqi-pick-trial-20261104.md)：候选高度下实际抓取、持物反馈与用户目视确认通过；扫描状态PARTIAL，未释放、未叠放。完整抓放配置仍有缺测；目标局部3mm净空、原ROS接入及静态几何报告保留各自范围。

[2026-11-03桌面扫描主控与17点实体记录](xiangqi-table-scan-20261103.md)：相机距桌面20 cm、停点采样与结束Home；人工8枚核对和提交前完整复跑，仍保留unknown/几何缺测与物理总数不确定性。

[2026-11-02项目发布记录](project-release-20261102.md)保存当前源码/运行身份、整包测试、Home切换、service生命周期及现场只读检查；下列旧批次数字保留原版本范围。

1. **机械臂恢复与反馈**
   - Home解耦代码阶段：提交`4ce936bd4bec11f8d34c641c4eac27a50258c402`，最终补丁SHA256为`72ed785945f4c55ddd6a504c1457e25d59c5073d5c74dd7b709faa85be1de199`。113 arm / 31 checkpoint / 20 twist / 12维护工具共176个C++用例、7个配置、27个roundtrip unit、13个history场景及维护工具/arm ROS/roundtrip/powercycle localhost协议回归通过；这些用例与全包测试有重叠，不能再相加充作独立总数。
   - 未修改旧serializer写档再由新实现恢复的兼容检查，以及真实checkpoint不可变副本检查均通过：原identity、local、previous_phase和端点保持，改变历史基准仍拒绝。真实副本检查不读取现场反馈、不覆盖原档、不执行现场恢复或动作。
   - Gemini通过Cursor CLI对最终补丁做独立静态审查，无剩余实质问题，主代理验收；测试执行证据由实现/集成方提供，审查者没有独立重跑全部测试。对应交付含`test-summary.json`、`legacy-proof.json`、`production-checkpoint-proof.json`、`peer-final-review.json`与`peer-acceptance.json`。本条仅说明代码阶段，现场加载和最终整包结果应另有发布记录。
   - [历史恢复与视觉解耦](history-vision-decouple-20261101.md)记录156项C++测试（107 arm / 29 checkpoint / 20 twist）及13个localhost历史场景；这些是对应源码版本的离线/模拟结果。
   - [10月31日1秒窗口](feedback-window-1s-20261031.md)的147项与3个loopback属于更早批次，不能和156项相加，也不能代表后续源码已重新测试。
   - 后续 `runs/lower5cm_feedback_20261101_01/delivery/summary.json`记录ROS状态发布约100Hz、原始反馈约122Hz、20/100Hz回环，以及一次约下降0.05033m的执行。位移来自关节和URDF FK，不是独立外部计量，也不扩展为完整运动范围验收。
   - 当前加载版本、Home目标与历史基准的关系以[日常操作](../operations/arm-quickstart.md)和实际运行配置为准；历史PID和安装前缀不作为新部署入口。
2. **象棋识别**
   - [小模型候选上线记录](xiangqi-lite-deploy-20261103/README.md)：YOLO26n全图候选＋原模型ROI分类/分割已部署，当前四子实测23.63fps；类别与截图一致99.22%、四枚深度几何同时通过93.94%，两者不等于总吞吐。
   - [11月2日候选/ROI与训练记录](xiangqi-precheck-20261102/README.md)保留旧方案约17.2fps及小模型离线训练阶段，深度预检测仍仅shadow。
   - 约35ms为保存四子数据的候选＋ROI阶段耗时，不是完整实时FPS；满棋盘、真实移动/遮挡和自动抓取尚未验收。
   - [节点拆分](xiangqi-node-split-20261101/README.md)、[严格FP32加速](xiangqi-acceleration-20261101/README.md)、[两秒稳定观测](../../tools/testing/xiangqi_tracking/STABLE_OBSERVATION.md)各有独立版本与范围。旧拓扑29.31fps不能当作当前双阶段策略结果。
3. **控制台、夹爪与标定**
   - [夹爪首版软件/只读](gripper-node-20261025/REPORT.md)、[11点实机收窄](gripper-sweep-20261025/REPORT.md)保留各自批次结果；未建立毫米开口、实际力和棋子夹持可靠性。
   - [10月31日原夹爪面板/初始化修复](manual-panel-20261031.md)保存瞬时值反例和独立浏览器/接口检查；后续六关节部署另记。
   - [网页控制台](../operations/web-panels.md)记录单关节/夹爪接口、历史模拟检查和现场只读检查；网页可用不证明运动或夹持可靠性。
   - [48观测与激光粗检](handeye-20261030.md)：36训练来自24位姿，12留出来自4位姿；留出中位0.208875mm、最大0.339257mm/0.111317°仅为内部一致性。
   - 激光粗检由用户确认可接受，真实夹指中点TCP和实体毫米误差尚未量化，候选外参未等同生产TF认证。

## 2 历史记录怎样使用

1. **保留原验证上下文**
   - 100ms失败与200ms续测分别保留；不能把旧运动结果写成1秒窗口资格。
   - 旧报告中的“当前/待部署/失败”均指报告冻结时的版本；现用入口见[日常操作](../operations/arm-quickstart.md)。
   - [历史恢复契约与三阶段结果](reference-recovery-history.md)合并被替代的实现阶段叙述，保留证据、SHA和失败修复事实。
2. **证据保存边界**
   - 本目录小型JSON、候选YAML、PNG与验收摘要保留；原始audit/图像/CDR和部署在被Git忽略的 `runs/`。
   - 历史报告不代表全工作区、安全认证、物理精度或当前版本实机验收；模拟与真实结果分开。
   - [过时文档入口迁移记录](documentation-migrations-20261102.md)列出6个已移除路径、替代入口及原文SHA；删除的是重复入口，不删除唯一验收数据。
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
