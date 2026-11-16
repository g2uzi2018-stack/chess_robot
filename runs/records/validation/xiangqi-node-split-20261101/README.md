# 视觉测量与场景节点拆分

记录日期：2026-11-01；改动基线 `be31b8d`。本次将源码默认节点 `/xiangqi_readonly_experiment` 拆为两个独立进程/ROS节点，不新增抓取执行器，不修改机械臂或夹爪控制逻辑。

```text
已有相机四路输入
    ↓
/xiangqi_perception
    ↓ /perception/xiangqi/measurements（String JSON，小型相机测量）
/xiangqi_scene ← 已有arm/joint反馈及被动审计
    ↓ /pose/xiangqi/detections、observation接口
已有预览；后续抓取执行程序
```

## 职责及接口

| 组件 | 负责 | 不负责 |
|---|---|---|
| `perception_node.py` / `/xiangqi_perception` | 同stamp RGB-D/内参/FrameInfo准入、原模型推理、相机系深度几何、每帧实例序号 | 不订阅arm，不读反馈审计，不读机械臂URDF/手眼外参，不维护基座ID/两秒快照，不发动作 |
| `scene_node.py` / `/xiangqi_scene` | 可信反馈、FK及基座变换、稳定编号、两秒不可变快照及当前资格、单次数据领取 | 不订阅大图/深度，不加载Torch/YOLO，不发动作 |
| `split_supervisor.py` | 两节点前台生命周期、任一子节点异常后的同组进程清理、绑定本任务的共享槽清理 | 不启动相机或控制驱动，不重启机械臂/夹爪 |

新主数据topic是 `/perception/xiangqi/measurements`，类型为 `std_msgs/String`，schema `xiangqi_camera_measurements/v1`。包含原SDK主机取帧 `stamp_ns`、相机坐标系、源epoch/sequence、RGB-D源帧键、内参、类别/置信度、框、相机表面中心/法向、质量字段、模型/工件身份及阶段计时。帧内 `instance_index` 不代表跨帧物理身份。

源消息最大1MiB、检测列表有上限，显示轮廓每目标最多128点。GPU二值掩膜、彩色图和深度仍在原三个有界共享内存槽内传递，不经此topic传输。同一保存场景全图/ROI的消息分别为5829/5881字节；这只是两个输入的实际大小，不是满棋盘上限。

场景节点继续提供旧消费者接口：

- `/pose/xiangqi/detections`：schema `xiangqi_measurements/v2`；顶层基座坐标语义及各有效位不变，稳定身份使用 `track_namespace + track_id`。
- `/pose/xiangqi/observation/request`、`response`：collect/query/consume。
- `/pose/xiangqi/observation/status`、`snapshot`：transient-local、depth1；当前资格看status，不根据旧snapshot消息仍存在就执行。

所有 `motion_ready/grasp_ready` 始终false。consume只记录一次数据领取，既不是执行动作，也不授予运动资格。同operation_id重试可以幂等返回已领取历史数据，不表示又准许一次动作。

可选反馈topic `/perception/xiangqi/roi_hint` 是短时提示，不是视觉启动前提。提示绑定已发布源epoch/sequence/stamp，缺失/过期/重启则回到全图；每18次提交仍全图搜索，采集两秒样本时强制全图，旧在途ROI结果不能作为新采集的起始样本。

## 时序、失效和编号

- 视觉无需机械臂或场景节点即可处理图像；场景可先启动等待，源恢复后继续。
- 保留原始图像时间戳，不用中间发布/接收时间冒充新测量。最终 `publish_latency_s` 从SDK主机取帧戳算到场景发布；不是曝光到动作延迟。
- 源重启/相机帧计数回退会更换epoch、退役旧身份；重复、乱序、超龄及不合格消息不进入采样。相机坐标从不填入基座字段。
- 静止区间内同一份仍符合新鲜度/停稳门限的反馈可支持多帧图像；反馈失联不能无限沿用。当前基座/FK不合格时历史快照可查询，但暂停新操作领取，后续合格新帧才可按原规则恢复。
- 模型、内参、手眼外参、URDF或策略身份变化会使旧快照当前资格失效。配置变更需按提示重启对应视觉组件，不能静默改变物理含义。
- 同一数据topic只运行一个有效perception发布者及一个scene拥有者，避免重复发布和身份竞争。

## 启动与退出

复用已有相机与安装环境；以下入口本身不打开控制器、不启动控制驱动或相机。已有实例时不要重复启动。

```bash
# 兼容旧入口：前台监督启动两个节点，保留TRT/ROS/路径环境
bash tools/testing/xiangqi_tracking/start_live_tracked.sh

# 或分别启动，便于单独接入与诊断
bash tools/testing/xiangqi_tracking/start_perception.sh
bash tools/testing/xiangqi_tracking/start_scene.sh
```

继续支持 `XIANGQI_SOURCE_DIR`、`XIANGQI_RUN_DIR`、`ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY`。使用TRT仍需原运行器、engine及局部依赖，并设置对应工件/源权重SHA256；未指定模型工件仍默认PT。单独scene入口使用系统Python，无需加载模型。

兼容双节点入口不接受把两个子节点重映射成同名的 `__node`/`__name` 参数；需要单独改名时使用各自入口。其他topic remap需确保两端一致。

正常停止或任一子节点退出时，监督器先给父节点清理机会，再清理本次独立进程组。共享内存兜底只接受本次token、perception PID和明确登记的三个共享槽，不扫描删除其他实例。正常SIGTERM返回0，子节点意外退出返回非零。生产运行期固定时钟仍应绑定整个前台监督器生命周期，不能在子节点启动后让外层提前返回。

单独裸启动不具备与systemd/双节点监督器相同的硬件驱动挂起升级保证；CUDA/系统完全卡死不是本次正常退出回归覆盖范围。

## 验证、独立审核及证据

完成一轮有界实现和定向修复；修复前16项审查复现为9失败、6错误，修复后16/16通过，另有原39项单元及隔离ROS回归通过。修复包括当前FK失败暂停新消费、JSON有限值/类型/深度边界、无效内参恢复、原图像时刻停稳门限、ROI真实来源时效及spin线程故障升级。原独立审核代理对最终SHA固定输入做增量复核，具体结论见 `incremental-result.json` / `incremental-review.md`。

Cursor（Claude Sonnet 5.5 High）报告已通过收集及范围检查，但这表示报告有效，并不表示初版代码通过。agy（Gemini 3.8 Flash Medium）的审查稿在预算到期前已写出，最终rename未完成；主代理只读收集并验证六字段稿，**没有peer工具正式验收**。前序agy High与Cursor Grok尝试未形成合格报告，不计为通过。两份peer意见经主代理核实、去除不可达或已有边界覆盖的项后纳入修复；不宣称所有工具都完整审核了全部代码。

初版验证包含16项原稳定观测、9项解耦、14项拆分逻辑回归，以及隔离domain116/117/120的图像准入、双节点两秒快照、场景先启动、无臂测量、反馈晚到、源丢失/重启恢复。三种进程场景（正常停止/perception崩溃/scene崩溃）都验证子进程组停止和三个共享槽回收。

同一个新保存场景按全图/ROI，使用相同缓存检测输出对照拆分前后：9个原始目标、8个去重目标，camera/base位置与法向最大差0。它不包含重新推理，不是独立标注集、实物定位精度或生产帧率验收。

新上下文审核子代理及peer-coworkers的agy/Cursor分别只读审核固定源码，主代理核实实质发现后修复。审查覆盖范围和实际CLI/模型身份、初次失败/恢复情况、各项发现与回归以本目录摘要为准，不能按模型票数裁决。

部署前的29.31fps属于上一版拓扑的四目标180秒实测，本次不能直接沿用为拆分后的帧率。模型、640推理尺度、严格FP32和质量门限保留，但新增topic调度与序列化开销必须另测。尚无拆分后满棋盘/长期热稳定/抓取闭环验收。

原始测试和审核证据在 `runs/xiangqi_node_split_20261101_01/`，冻结初版另有SHA清单；不将模型、原始图像或CLI账户/凭据写入Git。仓库只保留相关小型摘要、回归与代码。

## 本次生产切换

已仅重启原只读视觉service并切换到本次SHA固定源码，仍使用原TensorRT严格FP32工件及局部依赖。旧TOP入口已备份，部署helper含原路回退；没有新增开机自启。停服确认原54项时钟恢复后再启动，实际clock owner是前台supervisor MainPID，持久ExecStopPost release和45s timeout均已核对。

实际ROS graph出现 `/xiangqi_perception`、`/xiangqi_scene`，旧节点名消失；30秒只读订阅见两个topic均有真实模型完成输出。相机/渲染服务PID与状态未变，76个其他未提交路径及机械臂配置、命令审计SHA保持一致；未发送实体命令。

本轮30秒观察主要证明图像测量与旧消费者接口持续输出；当前反馈/停稳及几何资格按JSON单独记录，不构成实体基座定位、稳定ID或完整负载性能验收。观察JSON的valid_completion_hz仅为该30秒当前场景输出率，observer_receive_age_ms是SDK主机戳到观察者接收的时延；不是旧版publish_latency_s的同口径对照，不作为性能提升声明。

首次真实部署发现identity.torch的TorchVersion字符串子类未规范化，首个无效事件后持续有效消息被JSON发送边界拒绝；已回退旧TRT服务。修复只转换可信内部producer的字符串子类及NumPy标量，外部wire仍严格验证；修复后重新做了真实模型只读观察。失败观察与初版源码证据保留，未当作部署通过。

固定源码：`runs/xiangqi_node_split_20261101_01/source-final-02/`；生产输出：同目录`production/`。回退：先检查没有后续部署覆盖，再运行 `python3 runs/xiangqi_node_split_20261101_01/deployment_helper.py rollback`，只恢复原视觉入口。该脚本为现场绝对路径快照，不是新机器通用安装器。

预热后窗口30.011秒：旧输出接口900个不同完成帧（29.989次/秒），观察者另收到相机测量881帧；两topic初次发现时刻可不同，不能据两计数差直接宣称管道丢帧。窗口最后一帧检出目标数为0，相机几何有效为False，因此这仍不是有棋子完整识别性能验收。冷启动窗口另存deployment-observation-cold-start.json（包含加载/预热等待），不拿它与稳定窗口直接作性能比较。

