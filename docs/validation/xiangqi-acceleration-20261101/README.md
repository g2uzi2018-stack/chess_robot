# 保持 FP32 的识别加速部署与现场验收

记录日期：2026-11-01。基线 `facfcff`，Jetson AGX Orin 64GB、L4T 36.4.4、ROS 2 Humble、TensorRT 10.3。以下为断电前保存的实测，不表示设备当前在线或已经完成断电重启验收。

在现有 GPU/CPU 共享内存流水线上，采用 **TensorRT 严格 FP32 + 识别服务运行期间固定 CPU/GPU/EMC 时钟**。原模型、`imgsz=640`、`conf=.25`、`retina_masks=True`、几何门限、现有全图/ROI策略和两秒稳定采样门限保持不变。未采用 FP16、INT8 或降低分辨率。

## 现场前后对比

已有相机、识别、C++叠加与一个实际HTTP JPEG客户端同时运行；先测PT动态时钟30秒，再测部署后180秒，预热不计入窗口。不是随机交叉单因素试验，组合收益不能拆成独立贡献。

| 指标 | PT / 动态时钟 | TRT FP32 / 固定时钟 | 口径 |
|---|---:|---:|---|
| 采样窗口 | 30.06 s | 180.06 s | 墙钟 |
| 独立识别处理完成帧 | 423 | 5277 | 去重、检测有效 |
| 处理吞吐 | 14.07 fps | **29.31 fps** | +108.29%，不是四目标每帧都合格 |
| 四目标中心同时合格 | 12.01 fps | **19.89 fps** | 四个 `complete_center_target` 同时成立 |
| 采集到发布延迟 p50 | 149.86 ms | 92.22 ms | 主机SDK取帧戳到发布，非曝光到动作 |
| 采集到发布延迟 p95 | 186.09 ms | **108.11 ms** | 减少41.91% |
| GPU阶段 p50 | 54.22 ms | 20.21 ms | 包含检测调用，不是纯CUDA前向 |
| 几何阶段 p50 | 22.51 ms | 16.63 ms | 当前实机负载下墙钟 |
| 唯一HTTP JPEG实收 | 27.24 fps | 28.74 fps | 预览，不等于识别 |

吞吐由重叠流水线决定，不能用阶段耗时相加后倒数代替实测，也不能把29.31 fps解释为单帧延迟34ms。加速后基座变换通过5277/5277帧，但仅3582帧四目标中心同时合格；漏检仍存在。

## 四组实验的取舍

- **系统资源**：历史三轮离线试验，固定时钟吞吐中位18.52→22.81 fps（+23.16%），正确性签名一致。应用运行期固定时钟，不改风扇/CPU空闲策略，不切换现有MAXN模式。线程数改1仅约4%，不改默认。
- **推理后端**：TensorRT收益与资源频率、现场流程有关；前期动态时钟实机对照PT/TRT接近，固定时钟+TRT才达到约30 fps。最终以本次原流程现场验收29.31 fps为准，不把后端和资源收益相乘。
- **数据搬运**：GPU mask阈值化历史吞吐约+4.66%，尾延迟无稳定收益；另一方案约-2.66%。生产掩膜原本已是uint8，“再减少4倍字节”不成立，本次不采用。
- **DDS/QoS**：39-case实验没有证实识别吞吐收益；receive-only p95仅19.70→19.23ms，depth1还出现少序号。不改现有QoS，不宣称端到端零拷贝。

上述离线测试仅3个独立历史场景；不同harness与不同场景不能直接比较或合成收益。此次现场4目标也不代表满棋盘。

## 正确性、稳定采样和权限边界

- 新采集的一份RGB-D分别做全图/ROI的PT/TRT对照，共9个原始匹配目标，类别/匹配一致，mask IoU均1.0，框最大差6.1e-5px，几何坐标差0。只有一个独立场景，不是标注集精度验收。
- 180秒记录22个不同的四目标快照ID，每个56–61帧、2.0000–2.0646秒；最大采样间隔0.165秒，最低目标有效比例0.8。跟踪ID总计4个。
- 仍有3条 `needs_resample` 状态通知（场景变化或身份不明确），不能说锁定永不失效。目标基座位置窗口内径向p95最大0.943mm只是重复性，不是绝对定位精度。
- 所有 `motion_ready/grasp_ready=false`；类别、法向、yaw与实际抓取资格另行验收。识别无需机械臂参考/使能，基座测量和快照仍保留可信反馈门限。
- 本轮只切换识别服务。机械臂驱动、相机与渲染器PID未变；arm.yaml、安装前缀与动作命令审计哈希保持一致。周期checkpoint内容可正常更新。
- 原始16项稳定观测与9项解耦回归通过；Git入库前还在仅含待提交内容的副本重跑同样测试与跟踪自检。渲染C++包在同一副本构建通过，domain119隔离测试覆盖新鲜/移动/过期/无臂相机结果。测试不发实体命令。

## 代码、工件与运行期时钟

源码 `tools/testing/xiangqi_tracking/recognize_tracked.py` 支持可选 `XIANGQI_MODEL_ARTIFACT`（未指定仍默认PT）。启动验证 `XIANGQI_MODEL_ARTIFACT_SHA256` 和 `XIANGQI_SOURCE_WEIGHTS_SHA256`；源权重改变需重建并重新验证engine。运行记录增加后端、工件哈希、FP16/TF32状态，engine变更会使旧观测资格失效。

源PT SHA256：`f4fce2c0217b2ce292820e45f1779654b4cf58da3d156aff6306d56ae6aabf7e`。

engine SHA256：`26103821bc0788d1dc0a2d0051c91362530d5b6b1cc92347f2289d12c9469d35`。

engine为batch1，动态NCHW min `(1,3,384,640)`、opt `(1,3,480,640)`、max `(1,3,640,640)`，严格FP32、关闭TF32。运行环境Torch2.8.0、Ultralytics8.4.32。模型、engine、TensorRT局部依赖与原始图像不随Git分发，Git克隆后不能假定依赖齐全或自动运行。

部署目录：`/home/kuang/workspace/chess_robot/runs/vision_acceleration_20261101_01`。原视觉入口委托该目录的 `start_accelerated.sh`；服务使用该目录固定的 `source/` 副本，仓库两处后端改动与该副本哈希一致。原瞬态user unit名称仍为 `chess-robot-xiangqi-preview-20261031.service`。

`deployment-snapshot/` 保留**实际执行过的现场脚本快照**，不是通用安装器。脚本包含既有绝对路径与备份前置条件，不要直接在空目录执行 `deploy.py deploy/finalize/verify-lifecycle`。`performance.json` 是本次固定工件清单；启动脚本的固定值须同步，不能当作任意热切换配置。后端导出/构建源码在 `backend-build/`，环境和依赖仍需按记录准备，不自动下载/安装。

时钟由systemd activation持有，`ExecStopPost`调用 `clock_lease.py release`，保存并恢复原54项sysfs；先存后改，失败回退，持锁避免与离线实验同时改频。停服/再启动的恢复验证已通过。系统drop-in为 `zz-vision-acceleration.conf`，`TimeoutStopSec=45s`，避免原8s停止策略覆盖清理。没有新增开机自启，也不自动启动机械臂/夹爪。

运行期固定频率影响整机资源调度与功耗；本次210秒遥测CPU/GPU/TJ最高60.312°C，不能代替热稳定验收。`benchmark.lock`只暂停推理，不解除生产固定时钟；做动态对照前先停服或回退。

在保留原部署目录和备份的Jetson回退本次加速：

```bash
python3 /home/kuang/workspace/chess_robot/runs/vision_acceleration_20261101_01/deploy.py rollback
```

只恢复原PT视觉入口、原时钟与对应视觉角色，不回滚既有反馈/快照修复，不重启控制驱动。本次Git发布不重启任何服务；设备断电后还需要按现有人工流程启动，本文不声称完成开机恢复验收。

## 数据从哪里读

- `baseline.json`、`accelerated.json`：`completion_hz` 为独立识别完成帧数/窗口；`four_complete_center_frames / window_s` 为四目标中心同时合格速率。
- `latency_host_sdk_stamp_to_publish_ms` 为发布延迟分布；`live_stage_ms.gpu_stage/geometry_stage` 为阶段耗时；`http_jpeg_hz` 为唯一JPEG速率。
- 运行时 `/pose/xiangqi/detections` JSON中的 `stamp_ns`、`detection_valid`、`live_stage_ms`、`publish_latency_s`、`compute` 可核验处理与后端；不能只看网页VIDEO计数。网页识别Hz为短窗显示值，可能与180秒统计不同。
- `snapshot-validation.json`、`parity.json`、`lifecycle-stop-verified.json`、`final-verification.json` 分别是快照、同帧回归、时钟恢复与当时部署检查。
- 原始逐帧、arm/FrameInfo、HTTP序号、状态/快照、模型对照及遥测在远端部署目录。离场证据包SHA256为 `7b5fc112cfb3fdaa5b18042b71868bb8321d0c8d32202b6cd4117857333d5509`，12,303,812字节；本聊天outputs保存独立副本。Git仅保存小型摘要和脚本，不上传大模型/原始现场数据。

## 停止优化与后续验收

29.31 fps接近当前约30 Hz输入率，距30fps只有约2.4%的吞吐提升空间；这是本场景窗口结果，不是绝对硬件上限。停止零碎性能调参，优先解决漏检、验证满棋盘/标注集、长时间热稳定和断电后的人工启动恢复。未达到这些条件前，不写“满棋盘稳定30Hz”或“可直接抓取”。

简历可写：在Jetson AGX Orin上部署保持FP32与640推理尺度的TensorRT/资源调优方案，4目标现场180秒测试识别处理吞吐14.1→29.3fps，发布延迟p95 186→108ms，保留两秒稳定快照、工件校验与时钟回退机制。
