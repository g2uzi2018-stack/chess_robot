# 当前槽位就近选棋更新

2026-11-06，用户要求按当前摆放位就近选棋，并确认允许目标行有棋子。实现位于 tools/pick_stack/demo.py，入口为 row --workspace-ready。

当前槽位占位者优先；否则按当前槽位 XY 距离排序。未来槽位中的棋子允许作为来源，已完成槽位与退休源继续排除。占位者不可抓时，不抓无关棋子搬向该占位。粗扫点只用于调度，实际抓取与放置继续要求最新完整几何及实时空槽检查。已经准确摆正的棋子以两个独立视觉窗口确认，不闭爪、不下降。

离线验证覆盖距离排序、占位优先、占位不可抓的替代抓取拦截、未来槽位来源、完成槽位与退休源排除、原叠放选择保持、准确原位确认与错位拒绝。未生成持久测试源码，未使用冒烟测试。

实机只读前检失败：reference_valid=false、controller_mode=4、gripper.initialization=0；检查点 restorable=false，最后修改于 2026-11-05 21:40:56。用户确认断过电但没有手动拖动。找到的最新干净备份早于昨晚搬运，尚未找到覆盖中间全过程的恢复历史，未回退旧档、清零参考、使能、初始化或发运动命令。

另一条聊天当前做硬件 JPEG 与批量视觉优化，所查当前轮未见机械臂恢复写操作，不能把旧脏档归因于视觉线程。需要有效历史恢复或正式现场参考重建后，才能继续实机验证。

证据保存在 runs/row_nearest_slot_20261106_01/result-summary.json、selection-occupancy-validation.json、preflight01/preflight.json 和 readiness/arm-checkpoint-dirty.snapshot。此记录不声称新策略已实机成功。

已调用当前检查点恢复服务核对拒绝原因，返回 success=false，detail 为 history load/restore failed: checkpoint is dirty: prior motion did not reach a durable stopped save。此调用未发送运动目标，未恢复参考或使能。响应保存在 checkpoint-restore-response.json。

## 单命令启动准备（2026-11-06）

新增 tools/pick_stack/startup.py，由 run_speed_demo.sh 的 row/scan/vertical/prepare 复用。按序等待实测连续停稳、恢复现有可信检查点、按需初始化夹爪、确认空爪张开、受检回 Home、读回速度；失败不进入业务。activate 另在重启前确认持久化停稳存档。

启动命令：bash tools/pick_stack/run_speed_demo.sh row --workspace-ready --no-unobserved-motion。该声明只适用于停机期间没有手动拖动、碰撞等未观测运动的情况。有效参考/已初始化状态直接复用，不重置绕转或重复初始化。

内存验证覆盖恢复拒绝阻止初始化/Home、恢复先于初始化先于Home、已有状态跳过、持物/掉落阻止初始化、单次初始化成功及失败不重试。无持久化测试源文件，结果见 runs/row_nearest_slot_20261106_01/startup-validation.json。

实机仅执行 prepare --no-unobserved-motion，报告 runs/demo_startup_1793930154149085315/startup-result.json。取得停稳窗口后恢复服务拒绝 dirty checkpoint，状态 STARTUP_BLOCKED / RESTORE_ARM_REFERENCE；actions 和 gripper_actions 均为空，夹爪 initialization=0。未验证实机初始化成功、回 Home 或新占位策略整轮。当前缺失完整停稳记录仍需完整可信历史或正式现场参考重建，入口未绕过该限制。
