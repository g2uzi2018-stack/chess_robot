# 棋子跟随编号与平滑预览试验

2026-10-31现场：相机1280×720、请求30Hz、手动25000μs/增益16。用户要求编号跟随棋子移动，不绑定固定槽位。

## 行为

- Python实验识别与C++画面绘制分开：识别输入按同stamp配对彩色/深度/内参/帧信息，再匹配25ms内的实测关节；连续停稳0.6秒后计算表面位置。stamp为主机从SDK取帧时间，早于对齐/校正，不是曝光时刻；25ms条件不代表硬件曝光同步精度。
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

两个入口都不启动、使能或移动机械臂/夹爪；识别需要已存在的新鲜反馈，无反馈时只报告等待。现有相机继续复用，禁止重复打开USB。现场已运行的user service仍可使用`runs/`中的同等脚本；本次源码整理不重启它们。

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
