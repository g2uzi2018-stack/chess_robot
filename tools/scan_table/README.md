# 桌面扫描主控

`scan_controller.py` 按需启动 ROS 2 节点 `/xiangqi_scan_controller`，编排现有 `/arm/move`，订阅 `/pose/xiangqi/detections`。每轮正常结束回 Home 并确认实测停稳，然后退出；不注册常驻服务，不启动或重启 arm/视觉。

## 现场配置

| 文件 | 用途 |
|---|---|
| `scan.current.json` | 当前现场默认：相机距台面20 cm、17个离散观察点 |
| `scan.20cm.json` | 同一20 cm方案的命名配置副本 |
| `scan.home-height.json` | 保留的六点方案，观察点名义TCP约273 mm，非当前默认 |

配置绑定本安装、URDF/手眼/Home/历史参考/几何模型哈希，不是通用模板。实际台面 `base_link.z=-0.010 m` 已由用户确认。固定桌面轴约以前向 `-Y`、左向 `+X` 定义，完整向量在JSON；不随扫描姿态重定义。

设计窗口为前方12–80 cm、左右各40 cm；12×12 cm底座中心与基座原点重合，整条近侧12 cm是主动留出的区域。完整实体桌边尚未量测。20 cm指相机光学原点距台面，名义TCP距台面约10.25 cm；真实TCP、夹指净空与绝对误差尚未量化。

17点按远至近蛇形排列。近基座原网格两个点被关节余量/IK拒绝后改为左右补看，其中两个视角绕台面法向转90°，用图像长边向前覆盖近区。端点保持20 cm，PTP转移不保证全程等高。图像内缩32 px、按棋子高≤20 mm的包络再内缩18 mm后，设计窗口通过连续多边形覆盖核对；尺寸包络、遮挡与真实桌边仍需现场核对，不能由此承诺零漏检。

## 运行

沿现有项目入口建立ROS环境，只连接已有节点；输出目录必须为新目录：

```bash
source /opt/ros/humble/setup.bash
source runs/project_release_20261102_01/install/setup.bash
export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1
python3 tools/scan_table/scan_controller.py \
  --config tools/scan_table/scan.current.json --output runs/scan_NEW_PLAN
```

上述默认仅规划。用户已授权本轮实体动作并核对现场后，增加 `--execute`，改用新的输出目录。配置中的历史授权/设计元数据不代替本轮授权。

主控每段移动前从实际起点重新规划。Action SUCCEEDED、COMPLETED、有效实测终点及连续停稳分别检查。场景采样保留原0.2 s反馈新鲜度、0.6 s停稳、速度/差分/漂移与基座变换门限，不把arm的1 s反馈窗口拿来放宽scene。

当前模式 `proposed_wait_fresh_after_stop` 已实现：每点到位后至少停1 s，且至少收到3个停稳后的独立、新鲜合格scene帧；最多等3 s。缺帧时原地重采一次并保留两次证据，仍失败标记缺口，不无限等待。识别始终运行，unknown、几何拒绝和边缘不完整保留。正常完成（包括PARTIAL清单）先规划并执行业务HOME，确认实测到位停稳后才结束。

Ctrl+C只取消本节点拥有的Action，再单独证明停稳；取消回执不是物理停止。动作FAULT或停止未确认时不自动发Home，不用Home清故障，不清错/使能/恢复参考、不执行夹爪。应答超时不改门限、不自动重连或重放轨迹；已知FIFO排空、Stop回执和实测停稳须分别核对，后续动作必须使用新规划。

## 数据边界

状态 `/scan/xiangqi/status`、结果 `/scan/xiangqi/result` 使用String JSON。运行目录保存 `config.json`、`events.jsonl`、`scan-result.json`、逐视角scene原始测量与summary；运行证据在Git忽略的 `runs/` 中。

`ScanLedger` 暂由主控持有，复用scene已合格的基座测量，不重复实现YOLO、FK/IK或驱动。它不替换/消费scene原来的两秒不可变快照。短采样不能通过两秒重复性门限的目标保持unresolved，motion_ready/grasp_ready不放开。跨视角只标记可能重复；12 mm是搜索半径，未标定绝对误差前不强行合并，`unique_physical_target_count`保持null。单视角track_id不作为跨扫描物理身份，视野外未见不删除历史候选。扫描坐标只供后续预抓搜索，抓取必须重新测量确认。

限定实体结果、8枚现场核对及原始证据位置见[验证记录](../../docs/validation/xiangqi-table-scan-20261103.md)。提交/push不会部署或重启冻结视觉。
