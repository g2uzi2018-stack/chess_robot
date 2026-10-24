# 机械臂维护速查

在 AGX 工程根目录 `/home/kuang/workspace/chess_robot` 执行。
维护工具独立于 ROS 和业务节点，使用前确保没有其他程序发送运动。

```bash
# 首次构建；不需要启动 ROS
bash tools/arm/build.sh

# 清错、使能；非 TCP 时按厂家允许的状态请求切换
bash tools/arm/arm_prepare.sh --request-tcp --apply

# 回到保存的六轴 Home（不是六轴全零）
bash tools/arm/arm_home.sh --execute

# 示教到新 Home，静止采集并保存，旧文件自动备份
bash tools/arm/arm_capture_home.sh --apply
```

不带 `--execute` / `--apply` 时，只预览或保存候选，不执行运动/覆盖正式 Home。
正式 Home：`config/arm/home.yaml`；网络、速度比例和超时：`tools/arm/config/operator.yaml`。
回程路径需现场确认；Ctrl+C 会请求 Stop 并验证停止。采集不会重置业务扭转参考。

已使能且非 TCP 时，厂家协议不允许 RequestControl；先用示教器下使能或手动切 TCP。
没有独立的“在线模式”协议命令。详见 [完整说明](../../tools/arm/README.md)。
