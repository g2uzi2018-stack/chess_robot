# 棋子跟随编号与平滑预览试验

## 停稳审计计时修正（2026-11-04）

关节差分速度使用控制器毫秒时间戳间隔，避免网络批量收包导致误判；速度及新鲜度门限保留。1000帧停稳回放误判7→0，注入运动仍拒绝。现场以隔离源副本和临时覆盖加载；提交不自动替换冻结生产源码，详见[试抓记录](../../../docs/validation/xiangqi-pick-trial-20261104.md)。

## 共享ROI分组规划（2026-11-04，实验实现，未部署）

`XIANGQI_ROI_PLANNER=grouped`显式启用宽场景共享ROI分组，默认`legacy`保留原行为。`XIANGQI_ROI_GROUP_MAX_SIZE=640`限制新放大的ROI边长（480～704、32对齐）；704为待比较的较激进配置。该限制不缩小旧单目标ROI或已成立的单张联合/hint视图；原来可由一张ROI覆盖的场景保持原视图。

规划先按候选几何排序，贪心把可复用的目标加入已有ROI；需放大时保留原160px上下文预算，在尺寸上限内寻找不会新切穿其他候选的方形视图，并优先选面积增量小的组。不能减少ROI数量时保留原裁剪内容。输入顺序变化不改变分组，候选不因ROI预算被静默删除；8张预算与原融合/边缘/类别冲突/几何门限保留。

这不是最优集合覆盖算法，也不保证任意五颗棋子只用两张ROI。原消息中的四个分散候选浮点坐标，640配置仍需4张，704可减至2张（早期整数近似几何算例得到640为3张，但不代表原始布局）；最下方触及原图边缘的候选仍不合格。极端四角加中间在保留上下文预算时仍可需要5张。纯几何覆盖不等于语义通过：合成32候选网格中，新规划虽能降到8/4张，重叠视图的旧单目标裁剪仍可能产生边缘碎片歧义，完整分类保持false；未通过不能宣称32子识别验收。

该功能只改变lite策略的ROI规划；batch入口继续使用下一节配置。模型输入上下文改变可能影响类别与掩膜，应先同一旧batch1引擎比较legacy/grouped，再单独评价batch组合，不把同时换引擎和裁剪的结果当单因素收益。使用保存的旧四子RGB-D、明确标记的人工候选/调度算例，以及原验证集13张真实多子RGB图验证，不冒充真实分散8/16/32子、在线发布p95或物理定位精度。

旧四子12组RGB-D自然候选均保持1张ROI，legacy/640/704类别、掩膜、几何一致，原有unknown保留。精确历史散框仅作为人工候选负载，704可把4张减到2张、完整离线帧p50约133.5→90.4ms，但旧图真实棋子出现裁剪碎片unknown；这不是实际散子精度验收。

补充原模型val中的13张真实6/7/11子RGB图（130个标注目标、每模式两轮）：2D候选+所有ROI+融合p50约250.1/130.9/94.1ms，p95约262.6/161.5/155.7ms。按GT框IoU≥0.5一一关联，类别正确为54/72/87个（每轮130），完整场景正确各仅1/13图；新规划减少重复/切边碎片，但仍有大量unknown与额外实例，且个别图类别通过数下降。不得直接上线。该数据是训练时使用的验证集，棋子像素尺寸比原四子现场更大；没有深度，不能据此声称实体定位、现场小棋子质量、ROS延迟p95或Hz通过。原始JSON保留在`runs/xiangqi_roi_grouping_eval_20261104_01/`。

## 同帧ROI批量推理（2026-11-03，未部署）

源码新增显式`XIANGQI_ROI_BATCH_SIZE=2`或`4`；默认`1`保持原串行入口。只对lite策略ROI启用批量，候选仍每帧一次，不跨相机帧攒批，不改ROI规划/阈值/FP32/TF32与几何/快照门限。所有ROI返回后才融合并发布一个整帧结果。

现行原engine的batch固定1，不能直接设置2/4使用。`build_roi_batch_engine.py ONNX NEW_ENGINE REPORT --max-batch 4`从现有动态batch ONNX构建新外部FP32引擎，batch min/opt/max为1/2/4，保留height384～640、width640空间profile。新工件路径必须不存在；不覆盖线上engine，无需训练。构建和模型验证需使用既有benchmark.lock排他租约，生产会暂时让出GPU；不是线上部署命令，也不改时钟或服务。沿用既有局部TRT/CUDA依赖运行器，不自动升级依赖。

批量按兼容形状分组：任意源尺寸的正方形均变成640×640，可同批；矩形按原始height/width保守分组，保持每张独立rect=True时的最小padding。批量square显式rect=False与单张square原处理等价，单张detect始终显式rect=True/batch1，避免共享predictor参数残留。返回顺序恢复原ROI顺序，尾批1/2/3均使用合法动态batch，不填假图、不丢目标。

compute中`roi_count`与`classification_rois`表示ROI数，`roi_batch_sizes`表示各网络调用的实际张数，`roi_network_calls`表示网络调用次数；单ROI/候选数不能冒充完成帧数。旧batch1来源计数仍兼容验证。

本轮离线验证使用用后删除的临时脚本，原始结果与命令/脚本SHA保存在`runs/xiangqi_roi_batch_eval_20261103_01/`，汇总`batch-validation-summary.json`。实际归一化输入hash、类别/框、掩膜IoU、几何资格/中心及融合后输出均对照；12组原始四子数据自然规划均为1ROI，另用同图明确指定1/2/3/4/5ROI与矩形组验证批次/尾批，仅为调度负载，不冒充天然分散实测。14份暖后报告均输入hash一致、类别/数量/资格一致、mask IoU=1、融合中心差0。4ROI控制负载batch4完整离线p50为111.69ms，对照串行130.34ms，约14.3%缩短；5ROI为136.08ms对151.89ms，约10.4%缩短。单ROI约52ms基本无收益。小样本结果不代表实时ROS发布p95或最终FPS，也不足以宣称多目标掉帧已完全解决。

本节功能与离线一致性已验证，尚未部署；原冻结生产源码保持。仍需真实聚散多数量场景、实时完整链路与类别真值验收。

## 2026-11-03 当前生产：小模型候选＋原模型 ROI 分割

生产启动器显式选择 `XIANGQI_CANDIDATE_MODE=lite`，policy 为 `lite_candidates_roi_classes/v1`。每帧由 YOLO26n 单类 detect 扫描全图，再由原 YOLO26s-seg 输出同帧 ROI 的14类与实例轮廓；相机、scene、三槽流水线和原质量门限保留。

候选工件为外部 `yolo26n-piece-imgsz640-fp32.engine`，SHA256 `5ef87b3e4f6fd3991586845687a30ee8b5a0819019eb96cd73ebac1a80953a7b`。原图1280×720，TRT profile `[1,3,384,640]`，`rect=True`、conf=0.10，按分数降序 IoU>0.65 去重。该引擎不能用于任意正方形ROI，也不能替换原14类分割工件；两者均FP32、TF32关闭。

候选框保留在原图平面。只有唯一关联、且ROI完整覆盖候选框时，才能借用该同帧ROI掩膜做深度表面定位。无匹配掩膜的候选保持未知/几何无效，矩形框不能作为表面掩膜。冲突、裁剪边界、未关联候选、ROI额外目标和预算限制沿用原规则。消息附候选模型身份、调用计数与几何来源，候选模型不提供棋种标签。

配置：`XIANGQI_CANDIDATE_ARTIFACT` 为上述engine绝对路径，`XIANGQI_CANDIDATE_ARTIFACT_SHA256` 与工件一致，`XIANGQI_CANDIDATE_CONF` 默认0.1。源码兼容模式仍默认`yolo`；生产显式选择`lite`，不更换原`XIANGQI_MODEL_ARTIFACT`。

冻结源码、启动器备份和测量记录在 `runs/xiangqi_lite_deploy_20261103_01/`；项目发布启动器 `runs/project_release_20261102_01/deployment/start_visual.sh` 已切换。只重启视觉服务，沿用其时钟租约，不改相机、绘制或机械臂服务。perception与scene必须配套部署。

首次120秒聚拢四子实测：2838个唯一完成帧，23.65fps，两段一分钟分别23.63/23.67fps；每帧4个候选及4个输出、1个ROI。GPU阶段p50 35.53ms，其中候选7.84ms；SDK主机时间戳到发布延迟p50 120.40ms、p95 139.55ms。此窗口早于最后的多ROI掩膜覆盖检查修正，最终重启后60秒为23.63fps、1418帧，类别符合当前截图99.22%、四枚几何同时通过93.94%，完整配置/指标及回退见[小模型上线记录](../../../docs/validation/xiangqi-lite-deploy-20261103/README.md)。

当前摆放已与此前直线四子不同；类别符合截图、几何通过率与总吞吐分别统计，不能把23.65fps解释为每帧全部正确或完整验收。满棋盘、遮挡、绝对定位与抓取尚未验收；`motion_ready`、`grasp_ready`仍为false。以下11月2日段落为旧方案记录。

## 2026-11-01 当前节点拓扑

旧 `/xiangqi_readonly_experiment` 已拆为 `/xiangqi_perception` 与 `/xiangqi_scene` 两个独立进程。前者只输出相机系类别/几何和帧内实例序号到 `/perception/xiangqi/measurements`；后者维护基座变换、跨帧ID与两秒快照，继续输出原 `/pose/xiangqi/detections` 及 observation 接口。识别无需arm或scene也可启动，编号不下放到无状态视觉前端。

旧 `start_live_tracked.sh` 现前台监督启动两个进程；亦可分别用 `start_perception.sh`、`start_scene.sh` 启动。不要与现有实例重复发布。模型/严格FP32/640与质量门限保留；新topic传小型JSON，图像/深度/掩膜仍留在视觉内三槽共享内存。源码默认PT，TRT需现场工件及局部依赖。历史29.31fps属于旧拓扑，不是此次拆分后的完整验收。

详见[节点拆分、审核、启动与回退](../../../docs/validation/xiangqi-node-split-20261101/README.md)。后文历史段落保留其原时间与条件，以本节和新验收目录为当前入口。

2026-10-31现场：相机1280×720、请求30Hz、手动25000μs/增益16。用户要求编号跟随棋子移动，不绑定固定槽位。

## 2026-11-02 阶段记录

- 同帧全图候选＋ROI类别策略已部署，静止四子窗口约17.2fps；此前类别/几何及快照结果仅限该场景。
- 深度预检测仅为显式`depth_shadow`实验，真实数据有噪声/未决区域，未替换生产，见[深度配置](DEPTH_CANDIDATES.md)。
- YOLO26n单类预检测模型已训练，推荐640输入、conf=0.10、IoU>0.65去重。尚未接入生产；模型只输出框，不能替换原14类ROI分割模型。
- 同12帧的预检测＋ROI阶段约35ms，旧方案约48ms；不含几何/ROS/显示，不代表完整30Hz。
- 完整记录与模型SHA见[本次验收记录](../../../docs/validation/xiangqi-precheck-20261102/README.md)；训练/导出/评估源码见[训练复现目录](../../training/xiangqi_piece/README.md)。

## 2026-11-02 候选覆盖与类别证据拆分（已部署，验证范围有限）

拆分节点生产链现在每个独立RGB-D帧都先做全图棋子候选扫描，再用同一帧的ROI输出类别；首次两秒采样也执行ROI分类，不再等全图类别快照锁定后才启用ROI。全图候选的类别不进入检测类别、tracker投票或快照。现有分割模型仍在全图pass内部计算类别分数，并使用原`conf=.25`准入；这不是独立objectness检测，也不承诺发现任意物体或修复黑象漏检。

优先选覆盖本帧全部候选的单个方形ROI；宽场景使用按候选产生并去重的方形ROI，每帧最多8个crop。scene hint可选择分类上下文，但不能缩小全图扫描范围或遮住新候选。所有pass使用本任务同一共享槽、同一RGB-D采集stamp；掩膜和框映射回原图后仍通过原几何算法。模型、imgsz640、FP32/TF32策略和质量门限不变。每帧增加至少一次（有候选时）推理，30Hz吞吐、延迟、分类准确率和满棋盘覆盖尚未验证。

全图候选与ROI唯一关联后，候选的框、掩膜及表面几何仍使用全图本帧结果，仅借用ROI的类别与类别置信度，避免ROI重定位差异遮住全图看到的真实位移。只有ROI额外发现的目标才使用其ROI几何。多crop重叠处，同一目标的完整ROI观测可覆盖不完整边缘提案；两个完整观测有类别冲突时仍为unknown，不受crop执行顺序影响，也不能被后续干净同类观测抹掉。

同目标的完整同类ROI视图会保留到全图关联之后，再从也完整覆盖该全图候选框的视图中选择分类来源与对应置信度。最高分ROI若不覆盖保留的全图框，不会被误写为有效来源；没有兼容视图时保留全图几何但类别为unknown。选择来源不能消除完整视图之间已经出现的类别冲突，协议边界不放宽。

候选无匹配ROI、ROI类别冲突或裁剪边界目标保留为`unknown`，不沿用旧类、不投票、不授予快照资格。ROI额外发现的目标也进入本帧检测集合；crop或实例预算超限显式撤销覆盖完整性。所有帧使用`xiangqi_camera_measurements/v2`契约，携带`recognition_policy`、全图覆盖、分类完整性、同stamp分类来源和ROI清单。旧v1包及缺少类别来源的包不能进入新scene；旧scene也会按schema拒绝v2包。perception与scene必须配套更新，不能混版本部署。旧独立离线Experiment接口保留原功能，但其未提供ROI来源的输出不能作为新生产scene的合格测量。

scene只有在本帧全部候选具有ROI类别与完整相机几何时才采样/确认。语义或覆盖不完整会清空当前采样窗口；纯几何暂拒只跳过本帧并保留原max_gap/min_valid_fraction门限和独立帧分母。当前快照遇到漏检、几何暂拒、任意ROI类别冲突或场景位置变化会立即暂停资格；既有三帧不匹配撤销/重采样规则保留。采样中的类别变化重新采样；锁定前还检查候选快照是否匹配最后一帧，避免中位数/p95掩盖刚发生的移动。

`motion_ready`、`grasp_ready`及动作朝向资格始终false。此修改没有实体定位、法向/yaw、移动遮挡、满棋盘或抓取闭环验收含义。

离线回归：`python -m unittest discover -s tools/testing/xiangqi_tracking -p test_candidate_roi.py -v`。使用确定性模型输出替身，验证契约和状态逻辑，不验证模型准确率、TensorRT执行或实时吞吐。

历史单pass缓存不能冒充新协议的同帧双阶段证据。`compare_split_saved_inputs.py`默认会明确拒绝旧缓存生产链回放；显式`--legacy-geometry-only`只比较旧缓存的相机/基座几何公式，不验证新scene。`test_producer_scalars.py`继续验证旧缓存的标量归一化，并断言其缺少ROI来源而被新协议拒绝。需要重现旧拆分验收时使用其冻结旧源码，不能把历史脚本通过写成新链验收。

## 行为

- Python 实时检测独立运行；可信停稳区间内允许相邻图像复用同一实测关节状态。停稳后采样两秒，锁定不可变快照并供单一操作消费。接口、去重、质量门限、失效与历史语义见 [稳定观测说明](STABLE_OBSERVATION.md)。stamp 为主机 SDK 取帧时间，不是曝光时刻。
- 指定best.pt仍以imgsz=640推理；Home视角使用自适应局部裁剪，每18次识别执行一次整图侦察。裁剪/缩放坐标返回原始1280×720图平面；裁剪边缘不建立完整中心目标。类别是模型预测，Home视角下存在明显错分/波动。
- `piece_tracker.py`以基座下位置、有限速度预测与阵营不一致惩罚做全局关联；类别另经置信度加权投票稳定显示。新目标累计命中3次后确认，短时漏检不清零。编号单调递增、不复用，状态原子写入运行目录`track-state.json`。
- 一般关联窗口为60秒；启动时前三个含有效完整目标的更新帧，允许对超时旧轨迹按原位置5mm门限尝试恢复。近似同价关联标为不明确，不分配确定编号。相同外观棋子快速交叉或长时间遮挡仍可能失去身份，不能保证物理实例的永久身份。
- 每个编号的记录含本次观测stamp、当前XYZ、当前原始类别、累计预测类别、表面法向、倾角估计及法向离散度。圆形分割没有字面朝向信息，yaw为null、yaw_observed=false。深度法向在当前Home视角不稳定且无实体姿态验证，orientation_valid_for_motion始终false，不能直接转换成夹取姿态。
- C++包`chess_robot_tracking_preview`订阅原始校正彩色图、实测JointState和实验JSON；有新相机帧就绘图，不等识别完成。角点旧发布者不参与本次叠加。预览上限保持30Hz；绘制不是30Hz重新测量位置。
- 检测龄>0.5秒、关节变化>0.001rad、反馈龄>0.25秒或关节速度>0.002rad/s时，绘制程序清掉坐标/编号叠加，但实时视频继续。图片stamp保持当前彩色帧；JSON位置stamp保留其输入帧的主机取帧时间。

## 当前路径与启动

这是现场实验代码，默认工程路径为`/home/kuang/workspace/chess_robot`，没有自动安装依赖或下载模型。启动前需具备：

- ROS Humble，已构建`install/setup.bash`中的`chess_robot_interfaces`，以及`install_vision/setup.bash`中的视觉包；导入这些环境不会启动硬件。
- 外部模型`/home/kuang/workspace/xiangqi-yolo-training/20261031-yolo26s-seg-v1/runs/train/weights/best.pt`；SHA256为`f4fce2c0217b2ce292820e45f1779654b4cf58da3d156aff6306d56ae6aabf7e`。模型不随本仓库分发。
- 对应训练工程的`runtime/python-yolo`运行器，配置Torch/CUDA/cuDNN加载路径，并可导入ROS、NumPy、OpenCV、PyYAML、SciPy、Ultralytics。实测版本为Torch 2.8.0、Ultralytics 8.4.32；不要自动替换为当前最新版。
- 仓库中既有URDF、`tools/description/rigid_geometry.py`及候选外参`docs/validation/handeye-20261030/flange_T_camera_candidate.yaml`；只读联调仍须验证实际硬件标定。

按源码渲染启动器使用的前缀构建（从工程根目录）：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base runs/xiangqi_tracking_20261031_01/log build \
  --base-paths src/chess_robot_tracking_preview \
  --build-base runs/xiangqi_tracking_20261031_01/build \
  --install-base runs/xiangqi_tracking_20261031_01/install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

运行证据：`runs/xiangqi_tracking_20261031_01/`。

源码：本目录Python模块；C++包`src/chess_robot_tracking_preview/`。

只读识别源码入口：`bash tools/testing/xiangqi_tracking/start_live_tracked.sh`。

独立绘制源码入口：`bash tools/testing/xiangqi_tracking/start_tracking_renderer.sh`。

两个入口都不启动、使能或移动机械臂/夹爪；图像识别不依赖反馈，基座变换和新快照资格才要求可信停稳反馈。现有相机继续复用，禁止重复打开USB。现场视觉service使用固定源码快照，实际切换与回退见本次节点拆分验收记录。

现场两个临时user service：

```
chess-robot-xiangqi-preview-20261031.service
chess-robot-xiangqi-render-20261031.service
```

未设置开机自启；已存在实例不要重复启动。临时unit在停止后可能被systemd回收，需要使用上述脚本或重新systemd-run，而非假定start仍可用。停止这些实验服务不停止机械臂、夹爪或正式相机预览服务。网页入口`http://192.168.30.26:8088`选择“检测叠加图”。

## 验证与边界（以下为早期阶段记录）

- 本次用户授权下经现有`/arm/move`先plan-only再执行一次正式HOME，低速roundtrip_low。计划97采样、2.88秒；模型TCP从约82mm升到275mm；实测关节Home最大误差0.0000221rad，确认停稳。没有发送夹爪命令，没有重设参考、清零扭转或绕过业务节点。
- 局域网Windows客户端、独立JPEG序号、一请求在途：原彩色实测约20～22Hz；停识别后约22Hz；分离绘制后的叠加最后8秒实测约19.7Hz。配置上限30Hz，未达到稳定30Hz，未用重复图像补帧。现场主机到客户端流量不到1MiB/s。
- 四颗静止实物在多次类别变动下保持#1～#4，位置观测离散约0.6～1.5mm；这是短时一致性，不是定位绝对误差或身份跟随的物理移动验收。
- Python测试覆盖相同类别移动、输入顺序改变、类别跳变、短暂缺失与持久化恢复。C++绘制在隔离ROS domain119验证：新鲜观测显示标记、运动反馈/过期观测抑制标记且视频继续。测试不向真实设备发指令。
- 仍是实验识别/跟踪，不是正式完整位姿与自动夹取闭环。源码与文档尚未提交/推送Git。

## GPU与性能更新（2026-10-31）

模型一直使用CUDA GPU。本轮新增运行设备证据，仍为FP32；CPU掩膜和深度处理改为局部区域，GPU/深度计算独立spawn进程，无人订阅时停掉慢速调试图绘制/发布。预热离线单轮中位数111ms降至73ms，位置结果保持一致；实时10秒75次有效识别（7.5Hz），GPU推理中位数47ms，总发布延迟中位数198ms。尚未稳定30Hz。性能维护后已根据四个唯一近邻位置恢复原编号#1～#4，详细设备证据、阶段耗时、FP16未采用原因和维护审计见 docs/validation/xiangqi-gpu-performance-20261031.md 与 runs/xiangqi_gpu_performance_20261031_01/。

## 保持FP32的并行流水线（2026-10-31，中间阶段）

原模型、imgsz=640、FP32及阈值保持不变。当前使用shared_pipeline.py：一个GPU模型进程与一个CPU几何进程通过三个带锁和任务/stamp校验的共享槽并行工作，不再整帧pickle。9组输入/34目标同帧对照的类别、轮廓、有效性与坐标一致，最大位置差0。机械臂反馈有效时现场实测从7.4Hz升至15.6Hz，发布延迟中位数207ms降至168ms。完整精度和性能边界见docs/validation/xiangqi-parallel-20261031.md。

新识别服务退出策略为运行期KillMode=mixed、TimeoutStopSec=8s；主进程收到停止后等待工作进程退出，再释放共享内存。重启验证4.59秒，旧共享槽均释放。用户随后关闭机械臂时，不会启动/使能设备；基座定位等待新鲜有效反馈，相机预览独立继续。维护期间目标位置变化，当前新编号#9～#12，未强行映射旧号。

## 最终软件阶段与可复核证据

局部几何计算在完整掩膜紧包围框加零边界内执行；触及图像边界时保留原路径。3帧/9组输入/34个检测与36组合成测试通过，最大位置差约1.11e-16m；交替测试各216次，几何中位29.64→12.23ms。固定108帧回放18.13→18.98帧/s，反序复测结果接近。

相机保持720p30、原曝光/标定，校正预转换映射及4线程、视觉节点大图DDS共享内存后，暗场/无机械臂反馈时30秒四路各900帧同stamp完整交付；局域网唯一JPEG实收29.72Hz。此视频结果不是完整实机识别FPS，也不是长期可靠性认证。正常光照、实体位置、真实移动/遮挡与完整负载仍待验收。

仓库内摘要证据见[`docs/validation/xiangqi-performance-20261031/`](../../../docs/validation/xiangqi-performance-20261031/)。完整原始采样、模型、运行状态和构建仍保存在现场`runs/`及训练工程，不随Git分发。各历史段落的“当前/未提交”指其记录阶段，实际版本状态以Git为准。

CPU跟踪自检（无ROS发布）：使用上述Python运行器执行`piece_tracker.py`。叠加隔离测试见`src/chess_robot_tracking_preview/test/test_tracking_overlay.py`，其ROS domain固定119，不接触现场domain90。


## 2026-11-01 独立图像路径 / v2 资格字段

检测及相机表面几何不再要求 arm/reference/mode5；base 资格仍要求同步可信停稳反馈和模型/FK 校验。旧 detection.valid 仍表示基座几何合格，camera-only 使用 camera_geometry_valid；顶层 frame_id 仍为 base_link，camera 字段明确属于 d405_color_optical_frame。motion_ready/grasp_ready 始终 false。无 base 不提供稳定 base ID；原 track-state.json 不读取、不改写，v2 会话 ID 不跨中断继承。渲染支持预测/相机坐标及缺源恢复。详见 [稳定观测与资格说明](STABLE_OBSERVATION.md)。

启动脚本支持 ROS_DOMAIN_ID、ROS_LOCALHOST_ONLY 及 ROS remap 参数。识别另可设 XIANGQI_SOURCE_DIR / XIANGQI_RUN_DIR；渲染可设 XIANGQI_RENDER_PREFIX。脚本不会启动 arm、夹爪、相机或手眼驱动。生产常驻识别用 XIANGQI_BENCHMARK_LOCK 指向 runs/perf_ceiling_20261031/benchmark.lock，持锁覆盖在途 GPU/CPU 槽并周期让出；外层已持锁的隔离测试不设置该可选变量。


## 2026-11-01 TensorRT FP32 与运行期时钟加速

原模型、640推理尺度和两秒门限不变；现场顺序窗口PT动态时钟30秒14.07fps，TRT FP32/固定CPU-GPU-EMC时钟180秒29.31fps。发布延迟p95为186.09→108.11ms。四目标中心同时合格为12.01→19.89fps，不能把处理率当成每帧全部目标合格率。断电前已部署；没有新增开机自启，未在本次Git整理中重启服务。代码、工件哈希、四组实验取舍、原始字段、时钟恢复和回退方法见 [加速验收记录](../../../docs/validation/xiangqi-acceleration-20261101/README.md)。
