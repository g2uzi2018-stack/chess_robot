# 中国象棋快且准实施记录（2026-11-13）

当前版本已部署在 AGX Orin 64GB。整盘 32 子的旧生产链约 3.812 fps；本轮先改识别结构，再处理掩膜搬运、几何计算、JSON、跟踪和调度。最终十分钟识别与 scene 均约 29.76 fps，约为旧整盘的 7.8 倍。此处记录实际结果，不把工程目标写成已经通过的工业验收。

## 实现与边界

- 每个新 RGB-D 帧做一次全图分割和独立轻量候选扫描。候选取并集；缺少分割掩膜时仅在同一帧补查，不用框假造表面几何。
- 14 类字面分类独立于几何分割，所有棋子的小图批量进入分类器。红兵被旧模型误报红帅的问题，以用户实物确认作为依据，不拿旧模型作答案。
- 分割与分类器采用验证过的 TensorRT FP16 mixed 引擎，输入/输出仍为 float32；候选检测器保留原 FP32。分类阈值仍为 0.5，候选阈值仍为 0.1。
- 传递原分辨率的局部掩膜及全图偏移。32 子掩膜搬运约由 29.5 MB 降为 161 KB；没有降低几何所用的像素分辨率。114 组 RGB-D 的有效标志、轮廓一致，中心差最大约 1.6e-16 m。
- 稳健平面拟合的三轮数学运算移到 C++/Eigen；至少 50 点、2 mm RMS、深度范围等原条件不变。C++ 是 Cursor peer 实际交付，主代理完成编译和数值核对。
- 三个带所有权校验的共享槽、一个 GPU 进程、两个几何进程；允许 GPU 提前排一个新帧，完成结果按输入顺序交付。不会覆写正在处理的槽，所有输入仍受现有新鲜度约束。
- 移除生产中每秒主动排空流水线的 benchmark 协作等待。离线 GPU 比较必须先停识别，通过真实 systemd 单元借用既有固定频率租约，结束后恢复生产。
- 生产数据在来源处保持普通 Python 标量，对外发送时保留有限值、类型和大小校验；省去重复归一化与深拷贝。兼容入口的原 NumPy 标量转换保留。
- 跟踪关联和法向历史批量计算，轨迹仍逐帧原子保存，改用紧凑 JSON。100 帧回放的编号、位置、类别和资格相同；法向散布的浮点差最大 8.6e-7 度，无运动资格授予。

## 实测范围

最终连续实测（600 秒，启动预热另排除 15 秒）：识别 17,857 个有效独立新帧、29.7598 fps；scene 17,855 帧、29.7613 fps。所有帧均检出 32 枚；识别 17,714/17,857 帧（99.199%）类别齐全，余下保留 unknown，不能把“类别齐全”当作人工真值准确率。

识别/scene 分别有 97.33%/96.99% 的完整一秒窗口 ≥28 个新结果，达到本场景的窗口吞吐目标。主机彩色图 ROS 回调到结果接收 p50/p95：识别 89.13/127.99 ms，scene 133.05/177.07 ms；从原始 SDK 主机取帧 stamp 计，分别为 119.50/158.74 ms 和 163.46/207.87 ms。协议中的发布标记在 JSON 编码前，识别标记 p95 118.86 ms，不冒充精确 publish 调用时刻。**100 ms 延迟目标未达成；一小时与独立多摆放验收未完成。** 详见 [live-v10-summary.json](live-v10-summary.json)。

| 对比 | 结果 | 限制 |
|---|---|---|
| 相机源 | 30.002 fps，875 消息，硬件帧号无缺口 | 相机输出率，不是识别率 |
| 原整盘生产链 | 3.812 fps | 同一现场完整 32 子摆放 |
| v10 识别/scene（10 分钟） | 29.76/29.76 fps；一秒窗口 ≥28 帧比例 97.33%/96.99% | 单一完整摆放；延迟目标仍未达成 |
| v9 识别（2 分钟） | 29.972 fps；99.15% 一秒窗口 ≥28 帧；主机图像回调到发布前标记 p95 100.20 ms | scene 当时仅 27.09 fps，后续已优化 |
| 当前/稍后两批整盘存档 | 48/48 帧各检出并分类 32 子 | 同一摆放的重复采集，不是 48 个独立场景 |
| 旧内部验证集 | 198 目标均检出；195 类别正确、3 个边缘目标保留 unknown | 计入 unknown 的类别召回为 98.48%；不是独立工业测试集 |
| 旧模型同验证集 | 134/198 正确、214 个预测、51 个 unknown | 不能把旧模型输出当实物标签 |
| 新分类器单独验证 | 198/198 | 用于选择模型的内部 validation；不能宣称独立 100% 准确率 |
| 32 子三维表面资格 | 常见约 25～29 个通过 | 32 个识别完整不等于 32 个可抓取 |
| 外部实体定位与新完整抓取周期 | 未测 | 无外部 XY/高度真值；本轮没有驱动机械臂 |

工程目标为持续 30 fps、95% 一秒窗口 ≥28 新帧、主机图像入口到识别发布 p95 ≤100 ms、独立分类精确率与召回率 ≥99.5%、独立整盘场景成功率 ≥95%、持续一小时及外部位置精度验收。这些是待逐项达成的条件；当前一个静止场景的高完整率不能替代它们。RK3588 尚未移植或实测，不外推其具体帧率。

## 动作停顿改动

`tools/pick_stack/demo.row.live.json` 的逐目标重新观察，现为至少 6 个独立帧且跨度 ≥0.25 s；旧默认是至少 20 帧且跨度 ≥2 s。仅 row 抓取链使用该配置。目标重查、预抓取、重试、下降前再确认使用同一参数，其他业务及整盘锁定接口仍保留原默认。

停稳反馈、图像新鲜度、当前目标与邻居、未知目标、全视野覆盖、3 mm 重复性条件保留。896 个目标窗口的静态回放中，新旧平均位置差 p95 0.167 mm、最大 0.716 mm；这是窗口一致性，不是实体精度或实际动作节拍。短窗口已同步到现场，但本轮摆棋后未运动。此前空爪 Home/143 mm 往返属于上一个验证阶段，不重复计为本轮结果。

## 模型与复现

训练数据来自 2026-10-31 的既有分割标注：训练 190 图/785 目标、内部验证 49 图/198 目标，14 类。2026-11-13 摆放的整盘图没有进入训练。96×96 BGR、1.15 倍框范围；含旋转、缩放、降采样、模糊与光照增强。

原始执行配方保存在 [original-training-recipe-v2.txt](original-training-recipe-v2.txt)。当时 Torch/NumPy seed=43，Python random 未固定；因此冻结制品 SHA 是身份依据，不承诺按种子逐位复现。整理后的 `train_piece_classifier.py` 同时固定 Python random；未用整理版重新训练当前制品。

现场审计根目录：
`/home/kuang/workspace/chess_robot/runs/fast_accurate_20261113_01`

| 制品（相对此目录） | SHA256 |
|---|---|
| artifacts/fp16/roi-batch4-fp16.engine | 57542cda1c56d237d48f20bb8848d97ff7ab15b95864ac7bb7583619255de232 |
| classifier-v2/classifier.ts | d04312201a2f682f90d4175c8896a0cd3fcea04aacaafd41005dd916aa4a0ec9 |
| artifacts/classifier-trt-v1/classifier.engine | aabc945ff9bf4deec82a25fd1c353721071c7fd4d884e6dfa65d6cbedef7f0cb |
| artifacts/surface-v1/surface_fit.so | 389e4411c4471cb5648b930ee32e4a2c699c05bdf04c9e7d4a8508a1992c5b60 |

原分割权重 SHA `f4fce2c0217b2ce292820e45f1779654b4cf58da3d156aff6306d56ae6aabf7e`。独立候选 FP32 引擎 SHA `5ef87b3e4f6fd3991586845687a30ee8b5a0819019eb96cd73ebac1a80953a7b`。

构建入口：`build_roi_batch_engine.py`、`build_piece_classifier_engine.py`、`build_surface_fit.py`。运行模型保留在现场审计目录，不向 Git 添加二进制引擎。编译器 g++ 11.4、Eigen3，`-O3 -std=c++17 -march=native -fPIC -shared`，未用 fast-math。C++ 文件开头“未编译”描述的是 peer 交付时状态；后续主代理编译及验收证据见本目录。

## 部署与回退

现场视觉服务：
`chess-robot-xiangqi-preview-20261031.service`

最终启动器 `start_visual_v10.sh` 使用冻结 `vision-source-v8/`；每个源文件与模型 SHA 见 [vision-deployment-v10.json](vision-deployment-v10.json)。ROS_DOMAIN_ID=90、ROS_LOCALHOST_ONLY=1。保持既有 MAXN 和固定频率 acquire/release 生命周期，没有另加电源策略。

仅回退这次识别/跟踪版本时，将下列 drop-in 恢复为审计目录里的已保存版本，再 `systemctl --user daemon-reload`、重启上述视觉服务：
`~/.config/systemd/user/chess-robot-xiangqi-preview-20261031.service.d/zzzzzzzzzzzz-fast-accurate-20261113.conf`

`vision-v7-dropin.conf` 对应 v7（约25.7 fps，尚未提前排 GPU 新帧）；`vision-v9-dropin.conf` 对应 v9（识别30 fps但scene约27 fps）。perception 与 scene 必须整体使用匹配冻结目录，不能单独混搭。

机械臂观察窗口的原文件及 SHA 在现场 `pick-backup/manifest.json`；恢复该目录的 `demo.py`、`ros_io.py`、`demo.row.live.json` 到原 `tools/pick_stack/` 即恢复旧参数。没有自动启动抓取。

## 验证与 peer

执行了真实图像/深度回放、模型及坐标一致性、当前 ROS 新帧率/延迟测量、槽位所有权与异常释放的内存检查。没有进行冒烟测试，也没有新增持久化测试文件。临时序列化比较文件随 TemporaryDirectory 清理，JSON 报告是交付证据。

- Cursor Grok 4.6 Extra High、Fast OFF：实际交付短观察窗口及 C++ 稳健平面拟合；主代理独立检查、集成并验证。并行工作区 ownership 使其中一次 collect/accept 被拒，未绕过工具状态；实际代码仍由主代理验收。
- agy Gemini 3.8 Flash High：CLI 1.5.6 不识别 `gemini-3.8-flash-high`，提示会忽略模型参数；无法验证实际模型，因此没有向未知默认模型提交任务。不是完成的审查。
- 最后一次 Cursor 只读复核实际读过五个文件，但 360 秒内没有 handoff，已关闭，不能标记审查通过。
- 上述 peer 兼容性/交付问题已发给用户指定的修复聊天 `01a1ce39-eede-7463-b9bd-8685685970d9`。最终验收责任在主代理。

关键证据：`compact-pipeline-summary.json`、`native-producer-equivalence.json`、`tracker-batch-normals-equivalence.json`、`queued-dispatch-ownership.json`、`observation-window-replay.json`。完整逐帧记录留在现场审计根目录。

最终检查：31 个冻结生产源文件与当前 checkout 的规范化文本（换行、行尾空白和首尾空行）一致；18 个变更 Python 源文件语法解析通过，未执行源码或写入缓存；五个现场服务保持 active。启动期有 3 个过期输入，十分钟测量窗口无无效识别帧；未重启机器或发出机器人运动。
