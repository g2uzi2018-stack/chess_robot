# 扭转历史恢复与视觉解耦修复 — 2026-11-01

本页保留2026-11-01批次及明确标注的后续修复证据；“当前/未加载/回退”均指记录阶段。后续Home目标与历史基准已分开，旧部署/回退命令仅适用原批次，当前入口见[日常操作](../operations/arm-quickstart.md)。不要用旧报告覆盖新安装或改写旧测试结果。

本次修复基于现场未提交工作区；没有 reset、stash、提交或推送用户修改。原始清单、源文件归档和任务差分保存在 `runs/history_vision_decouple_20261101_01/`。

## 根因与证据

- 现场历史 checksum 正确、restorable=true；只读窗口 74 条 ArmState 均 reference=false/mode4。当前关节距存档端点最大差 0.0025972146 rad，低于原 0.005 rad 门限。离线按现场参数重算 fingerprint 并用副本成功加载、校验完整展开几何；原 total=0.29670920672624873 rad，signed=-0.29670627724905013 rad。离线 join 不清零；这不等于实体参考已经恢复。
- 命令审计最后失败项是 GetErrorID() 的 Connection reset by peer，不是已经证明 EnableRobot() 被发送。旧 recoverCheckpoint 在 restoreCheckpoint 提交历史后执行可选使能；任何后续异常都会 invalidate 已恢复历史，并禁止自动重试。错误诊断因而把历史提交与执行器失败混在一起。TCP 重置的底层原因没有证据，本次未重发命令探测。
- 原周期保存同时在 State 和 CheckpointStore 限定 mode5，导致停稳 mode4 不能持久化。旧视觉入口又要求 reference=true/mode5 才 submit，故四路图像正常仍完成零次推理。
- 现场运行 prefix 是 `runs/joint_panel_20261031_01/install`，其安装二进制 SHA256 为 `70e31dc823ae14d4b0d8c8ee32acabf4242f48c09183ebd9681bd08a2d811784`。安装构建快照与工作区并非逐文件一致；恢复函数中的上述失效逻辑一致。不要把当前源码等同当前运行版本。

## 历史恢复状态机

| 事件/条件 | 历史资格 | 执行器结果 |
|---|---|---|
| 坏 checksum、dirty、身份/模型 fingerprint 不符、端点不符、反馈不可信或 dwell 不足 | 拒绝提交 | 不发命令 |
| 干净完整存档 + 配置绑定一致 + 用户断电不移动前提 + 新鲜连续停稳 mode4/5 + 端点匹配 | 恢复原展开相位与扭转，不清零 | 默认纯恢复；mode4 不需要先使能 |
| 配置显式选择恢复后使能 | 历史已提交后独立处理 | GetErrorID/EnableRobot 原有约束不变，准入独占，无自动重试 |
| 使能阶段 TCP 重置、超时或拒绝，但随后收到新的、同一参考 generation、连续停稳且端点匹配的 mode4/5 反馈 | 可保留已验证历史 | 单独报告 enable failed；不冒称成功使能 |
| ACK 后未见 mode5 | 连续可信历史可保留 | 超时报告，不自动再发使能 |
| 反馈 gap/epoch 回退/跳变、真实运动破坏停稳条件、端点漂移或无新确认 | 对应历史 generation 失效 | 报告确认不可信；旧完成回调不能抹掉较新显式恢复 |
| 活跃参考丢失 | 失效，阻止本进程自动反复恢复 | 等待明确恢复，不用存档掩盖异常 |

纯 checkpoint 恢复不发设备命令；旧审计历史恢复也复用同一使能确认策略。RestoreCheckpoint.success 表示历史是否仍有效，若可选使能失败但历史保留，success 可为 true，detail 和 ArmState 的 actuator 诊断明确失败。不能把该 success 当作 motion_ready。

周期/动作后保存独立于相机与渲染；mode4/5 均必须新鲜、有参考、停稳 dwell、无活动任务/故障。保留 SHA256、完整局部展开相位、配置 fingerprint、单 writer、随机临时文件、文件 fsync → rename → 目录 fsync。持久化期间持有任务准入，避免旧 clean 快照覆盖新动作 dirty 标记；仅在落盘返回成功后记录保存完成。已观察运动即使参考无效也尝试写 dirty；失败会失效并明确诊断。受控动作仍在发送前持久标脏。

这里的身份绑定是现有控制器地址/端口、协议/配置固件、fixture 标记、URDF/Home/TCP/展开配置的 fingerprint，不是硬件序列号认证。不得据此声称能识别被替换成同地址、同配置的另一台控制器。控制器替换或身份不确定时禁止沿用旧存档；应另行建立实际设备身份绑定。异常断电/运动中未提交存档没有自动放行承诺。

## 视觉接口与兼容

JSON 增加 schema=xiangqi_measurements/v2。旧顶层 frame_id 保持 base_link；camera_frame_id 明确为 d405_color_optical_frame。字段名中的 camera/base 坐标系不变，不将 camera xyz 填入 base 字段。

| 层/字段 | 含义 |
|---|---|
| detection_valid | 本帧模型预测完成且图像有效；不是人工类别验真；不依赖 arm/reference/mode5 |
| camera_geometry_valid | 同帧有效 RGB-D/内参且原深度质量门限通过 |
| base_transform_valid | 同步新鲜、非 simulated、非 busy、连续停稳 mode4/5 关节 + URDF/外参有效 + FK/TCP 一致；只读测量 |
| detection.base_geometry_valid、旧 valid | 该检测的基座几何合格；保留旧 valid 语义 |
| motion_ready、grasp_ready | 本实验始终 false；不授予运动资格 |
| complete_center_target | 仍是基座几何且完整目标；新增 camera_complete_center_target 服务图像 ROI |
| track_namespace + track_id | 当前观测会话的基座轨迹 ID；跨会话或资格中断不承诺继承 |

无基座时不提供稳定基座 ID，不调用基座三维 tracker 做 camera 关联。原 `track-state.json` 不读取、不修改；每会话另存 v2 文件。反馈 generation 或观测空窗变化会退休旧轨迹，next_id 不在会话内复用。运动/参考变化只撤销依赖 arm 的基座结果；保留新鲜检测和相机几何。超龄图像撤销相机/基座资格。渲染可显示 camera-only/类别预测，源缺失时等待，过期清除，源恢复后继续。

FP32、imgsz640、conf .25、RGB-D 同 stamp、图像年龄、全部深度几何门限不变。三槽 ownership/job_id/stamp/有序回收保留，补齐异常释放。生产推理可用 XIANGQI_BENCHMARK_LOCK 与公共 flock 协作，短批次完成并排空槽后让出；离线测试由外层持锁覆盖子进程清理，避免争抢资源。

## 独立启动矩阵

| 启动项 | 真正依赖 | 上游缺失时 |
|---|---|---|
| arm 节点 | 控制器反馈、模型、合法历史/原有运动安全配置 | 相机缺失不影响历史保存；运动资格照原门限 |
| 独立 vision.launch.py | D405、有效相机配置 | 不需要 arm；未拿到 USB/图像则自身等待/报错 |
| live_tracked | RGB-D/CameraInfo/FrameInfo、模型 | arm 缺失仍检测和相机几何；图像缺失等待 |
| 基座变换/基座跟踪 | 合格关节及时间匹配、URDF/外参/FK | 不发布旧 base 充数，不生成稳定 base ID |
| tracking_preview | 图像；可选识别/关节 | 无识别先显示视频；无 arm 显示预测/相机坐标；源恢复自动继续 |
| camera_preview.launch.py | 相机 + LAN 预览 | 原 launch 没有 arm/夹爪/手眼节点依赖；本次无需改动相机驱动 |

`start_live_tracked.sh` 与 `start_tracking_renderer.sh` 只启动各自组件，支持环境覆盖和 ROS remap；source 安装环境不等于启动控制节点。不要使用带实体自动使能的组合 launch 做测试。

## 回归与现场边界

156 项 C++ 单测（107 arm + 29 checkpoint + 20 twist）、9 项视觉 CPU 回归、13 个 localhost/domain118 机械臂 loopback、domain119 渲染集成通过。另有 domain117 图像准入集成覆盖 RGB-D 错 stamp、超龄图像、无效内参、无 arm 合法图像、无效深度、stream 失效及源恢复七类反例；使用确定性几何夹具，不把它计为真实模型测试。模型同输入对照的 boxes/masks 及 4 个目标的 camera/base 几何完全一致。现场候选 remap：无 arm 8 帧新鲜相机几何；reference=false/mode4 时 8 帧新鲜相机几何中 7 帧 base 合格；晚接入/丢失/恢复时检测继续，基座按层恢复，ID 不跨未观察区间继承。另在 mode4、mode5 各执行一次同一 checkpoint 的完整节点进程关闭/重启，原非零 signed/total 保持在 1e-12 rad 内，命令列表为空。最终数字以 runs 下 release JSON/XML 为准。

离线可复现入口：

```bash
bash tools/testing/run_history_vision_regression.sh   /home/kuang/workspace/chess_robot/runs/history_vision_decouple_20261101_01/source-before.tgz
```

入口只构建离线快照并运行 localhost/domain118、119 测试，不启动实体节点/相机，不发真实控制器命令。原始日志保留初次测试夹具没有模拟反馈重连所致失败，以及修正后的通过结果；没有把该夹具失败归咎于实机。

本次仅允许独立更新只读视觉服务。arm/gripper/handeye 未停止、重启或加载新二进制；没有 EnableRobot/Home/移动/清错/夹爪/恢复服务调用。实体掉电和修复后机械臂加载仍需另一次明确的现场验收。建议下一次加载前把 checkpoint.enable_after_restore 显式设为 false，以纯恢复验证非零扭转，再单独决定执行器操作；本次没有修改现有 arm 参数。

视觉当前发布来源、自动回退及加载后证据见 runs 下 deployment-02/，固定 Python 快照为 vision-source-02/，渲染二进制来自本任务 install。第一次 deployment/ 因 source 安装环境将 ROS_LOCALHOST_ONLY 设为 0 导致接收资格失败，已自动回退；启动脚本现先保存指定值/默认值、再 source 并恢复，两次服务切换均未操作 arm。现有两项识别/渲染 unit 是 transient unit；本次保留该生命周期，设备重启后的启动编排没有改成自动启用。独立脚本可单独启动各模块。回退只恢复两项视觉 wrapper 并重启对应视觉 service，不操作相机或控制器。代码回退应使用任务前归档/任务差分，不能用 git reset 清掉用户已有改动。

当前视觉回退入口（只涉及两个视觉 unit，恢复任务前 Python 快照和旧 renderer）：

```bash
python3 /home/kuang/workspace/chess_robot/runs/history_vision_decouple_20261101_01/deploy_vision_02.py --rollback
```

首次部署前后，arm PID 2909620、arm.yaml、arm-checkpoint.save、arm-install-prefix、旧 track-state.json、commands.csv 的哈希均一致。常驻识别通过短批次 lease 配合公共 flock，完整离线回归入口实际重新构建并跑通；相机视频在基准持锁期间继续，识别等待并在释放后恢复。
