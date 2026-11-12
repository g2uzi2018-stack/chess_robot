# 2026-11-07 下探前场景就绪等待修正

用户这轮记录 runs/row_speed_20261107_091957_181080397/run-result.json：启动正常、完整扫描，前3个占位槽跳过；本轮新放0颗。candidate-8 与 candidate-9 的下探规划完成后，执行前 fresh stationary full-scene evidence 检查失败，未实际下探；candidate-11 因真实邻近几何进入抓取包络被跳过。最后空爪Home、参考有效。

## 根因复现

只规划任务也会使 arm/state.busy 短暂变化。视觉 FeedbackWindow 把 busy 作为资格身份之一，任务结束后需重建连续静止资格，再等图像与处理管线跟上。旧 fresh_view 是单次瞬时检查。

现场仅规划复现：规划前视觉合格，规划后机械臂已满足 Controller.stopped，但最新场景仍 base_transform_valid=false、reason=settling，图像年龄仅约0.175秒；再等约0.238秒场景恢复合格。记录 readiness-reproduction.json，未发送物理运动目标。

## 修改

仅修改 tools/pick_stack/ros_io.py。新增 wait_observation_view，规划前与实际发送前均有界等待新鲜、完整、姿态匹配、身份一致的场景，最长使用原 observation_timeout_s=6秒。原图像年龄0.5秒、反馈年龄0.2秒、静止速度0.002rad/s、关节漂移/匹配0.001rad、TCP起点3mm要求不变。参考/反馈或身份异常、中断仍立即退出，超时不会发送运动目标。

observation_scene_waits 记录等待时间、图像年龄、覆盖、变换资格、拒绝原因和关节差；下探记录增加 PLANNING / APPROACH_ABORTED 等结果，避免只有目标却无法判断有没有执行。没有放宽110mm抓取净空，没有修改视觉模型或服务，也没有重启或重建参考。

## 验证

7项针对性内联检查通过：settling后就绪、持续无效超时、参考丢失、中断、身份变化、陈旧图像，以及超时前不能发送物理目标。未生成持久化测试源。

修正后现场仅规划再次复现：旧瞬时检查点仍为 settling；新等待约0.143秒后 READY，图像年龄约0.148秒、关节差约0.0000015rad。physical_motion_goals=0、gripper_goals=0，机械臂保持Home。实际“下探后恢复识别并完成抓取”的闭环尚未重新执行。

证据目录 runs/observation_ready_20261107_01，包括 readiness-reproduction.json、behavior-validation.json、live-validation.json 和 result-summary.json。
