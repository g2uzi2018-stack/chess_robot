机械臂既有检查与工具提示

只在修改对应行为或当前任务确实涉及它时执行。纯配置注释、README 和排版修改不触发测试。
不运行冒烟或 mock；不默认运行整套 test_arm、test_checkpoint 或 scripts/validate_target.sh。
该旧脚本会启动整套历史测试，其中有设备替身，不能用来代替真实设备验证。

已核对可独立使用的纯解析/换算用例
文件：test_arm.cpp
- Protocol.EchoErrorsAndNestedPayload
  保留理由：命令解析需要长期区分错误码、嵌套内容和原命令回显，防止误认另一条命令的回复。
  修改 dashboard_reply.hpp 的解析规则时使用；直接测试实际解析函数，不连接或模拟控制器。
- Protocol.RotationUnitsAndBranchBoundaries
  保留理由：mm/deg 与 m/rad 的转换、接近 ±180° 的旋转表示会直接影响运动目标。
  修改 SDK 单位/姿态转换时使用；直接测试实际换算函数。

上述用例需在已配置 BUILD_TESTING=ON 的构建中编译 test_arm。
按当前任务选择一个过滤项，从项目根执行，例如：
  build/ros/chess_robot_arm/test_arm --gtest_filter=Protocol.RotationUnitsAndBranchBoundaries
不要删除过滤项后改跑全套，也不为说明这些入口而执行它们。

其他现有内容
- test_arm.cpp 其余用例：混有替代运动学、控制器回环和人工反馈；不能按文件整体认定符合新规则。
- test_checkpoint.cpp：覆盖存档完整性、完整圈数、运动前标记及日志轮换。
  这些行为有长期核对价值，但文件也混有设备回复替身和人工反馈；不运行整套，
  不以存档检查通过宣称设备恢复已验证。涉及对应实现时再选择符合当前规则的部分。
- replay_feedback.cpp：读取真实 feedback.bin 的离线解析工具，不是设备模拟器。
  用途是排查已有反馈记录中的重复时间戳/关节变化；仅处理当前问题涉及的原始记录。
  已构建入口：build/ros/chess_robot_arm/replay_feedback <原始反馈文件>。
- migrate_checkpoint_offline.cpp：按指定原始记录和迁移计划写入历史存档，有实际文件副作用。
  仅在明确的历史迁移任务中使用，不属于节点完成后的常规测试；详细约定见包 README 指向的恢复说明。
- restore_reference_offline.cpp / checkpoint_fixture.cpp：前者循环刷新离线快照，后者构造模拟存档；
  属于旧模拟验证入口，按当前不使用 mock 的约定不执行。

本文件没有将全部旧用例认定为需要长期保留；源码仍在原位，不在外围文件整理时批量删除。
