# 2026-11-06 可信参考恢复与自动启动

结果：参考有效、夹爪初始化完成、空爪 Home，速度仍为 0.5/0.5 rad/s，加速度 0.3/0.1 rad/s²。未启动扫描抓取。

## 旧记录为何不能恢复

原 .runtime/arm-checkpoint.save 于 11 月 5 日 21:40:56 被观测速度尖峰覆盖为 restorable=false。保留的原始反馈显示：末尾健康模式 5 速度约 0.023 rad/s，随后 2 帧模式 9，次日模式 4；最大端点变化 0.002637 rad，在既有 0.005 rad 范围内。不能直接改原记录的布尔值或回退早于后续运动的备份。

## 一次性恢复证据

旧 arm_node 进程 3302180 尚存完整 Tracker 的 joints、previous_phase 和 local。只读调试提取这些字段，没有修改进程的有效标志，也没有在调试器中调用控制函数。临时调试类型源已删除。留存数据与原始反馈逐轴精确匹配；此前连续停稳 1.220283313 秒。

通过原始相邻反馈逆向重建最后停稳状态，再正向核对局部绕转；校验实际故障后缀、断电接续、全部关节限位和局部绕转，直接端点接续与完整原始轨迹的局部绕转一致。使用原 CheckpointStore 编码并校验模型身份，再经原 restore_checkpoint 服务恢复。未清零历史或直接采用旧备份。

首次使能明确成功但旧即时确认遇到抱闸速度过渡，错误地使参考失效。随后独立采集 mode=5 连续停稳证据，再执行仅软件参考恢复，没有重复 EnableRobot。夹爪 INITIALIZE 完成，受检回正式 Home。

证据目录 runs/reference_recovery_20261106_01：retained-state.gdb.txt、retained-tracker.json、retained-endpoint-feedback.bin.15.bin、offline-reconstruction.json、original-dirty.checkpoint、reconstructed-stopped.checkpoint、post-enable/state.json、prepare-confirmed-mode5.log。原始帧保留未改，SHA256 记录在 retained-tracker.json。

## 后续自动启动改进

run_speed_demo.sh 自动启动或重建同名机械臂服务，然后恢复参考、按需初始化、确认空爪并受检回 Home。新增 shutdown：确认持久停稳存档后停止机械臂节点，避免随后关机反馈覆盖存档，输出 POWER_OFF_READY。

使能确认仅在明确成功的 EnableRobot 后允许连续可信反馈内、原 0.005 rad 端点范围内的短暂稳定过程；等待新的 mode=5 连续停稳窗口，不重复使能。失败或不确定应答路径保持原严格验证；异常模式、反馈失效、超限或等待超时仍拒绝。

核心改动位于 src/chess_robot_arm/src/core.cpp、node.cpp 和 include/chess_robot_arm/core.hpp。现场二进制部署在 runs/demo_startup_20261106_01/arm_node，由现有 start-arm.sh 加载，参数与几何守卫不变。

## 验证范围

31 项原检查点验证及 3 项新增针对性验证全部通过：抱闸过渡后恢复且保留大于 pi 的完整局部绕转；模式 4 不误判模式 5；漂移、Servo 模式、反馈缺口仍拒绝。测试源和二进制均在临时目录，结束自动删除。日志 enable-settling-validation.log。

实机通过：初始化和 Home、修正后受控重启恢复、shutdown 保存可信记录并停止节点、prepare 自动重建临时 unit 并恢复到 READY_HOME_EMPTY。日志 orderly-shutdown.log、automatic-start-after-shutdown.log，最终摘要 final-readiness.json。

未再次真实断电，新的模式 4→使能过渡仅离线验证；未重新执行占位优先摆行。视觉线程未修改。突然中断运动或未知拖动仍不自动绕过恢复检查。
