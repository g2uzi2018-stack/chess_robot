# 扫描后逐枚抓取、Home下方裸桌叠放 demo

2026-11-04：真实 ROS 执行适配已接入，默认三枚。独立受监督试抓已完成：另一位置红卒在候选名义TCP基座Z=-0.005235732m处抓取、竖直抬升约20mm并停稳，用户目视确认稳定。尚未松爪或叠放；完整抓放配置中的未量测值仍为null，缺测时在启动张爪、回Home完成后的抓放前检退出，尚不进入扫描或抓放。详见[单颗试抓记录与证据](../../docs/validation/xiangqi-pick-trial-20261104.md)。

`grasp_trial.py`是独立的受监督单枚抓取工具，结束于持物抬升保持；`observe_contact_point.py`和`observe_vertical_step.py`用于候选接近/竖直观察。它们使用现场候选安装与独立参数，不表示默认`run_once.sh`已能完成整套叠放。候选高度只经该位置单次验证，不能当作所有棋子的TCP标定。

## 接口与流程

| 接口 | 用途 |
|---|---|
| `/arm/move` / `MoveArm` | 每段先只规划，再从实际起点执行；PREGRASP、DESCEND、RETREAT、正式HOME |
| `/gripper/command` / `MoveGripper` | 只用MOVE；新夹持require_contact=true，不发INITIALIZE |
| `/arm/state`、`/arm/joint_states`、`/gripper/state` | 实测停稳、参考、终点、夹持/掉落与释放核对 |
| `/pose/xiangqi/detections` | 停稳后新鲜独立场景帧，不改原scene快照/ready字段 |
| `/pick_stack/status` | 有限流程状态、层数、物体状态，String JSON |
| `/pick_stack/confirmation_request` | 当前人工观察请求，String JSON，transient-local |
| `/pick_stack/confirmation` | 人工确认回复，String JSON；必须匹配task/request/kind/layer |

流程：设备前检→启动张爪并确认张开→正式Home并证明停稳→抓放前检/人工确认控制权与裸桌空位→现有扫描→正式Home并证明停稳→选搜索种子→观察位重观→预抓对齐→再次重观→抓取下降→新夹持→抬升→固定Home投影预放→下降→确认承托→释放→撤离→观察堆顶→确认放稳→提交层数/退役源位置→下一枚→正常正式Home。

`scan_controller.Controller`只增加可选node_name参数，原独立扫描行为不变。demo不创建驱动、RS485或SDK连接，不启动/重启服务，不改参考、碰撞门限、手眼或生产视觉。源代码修改不等于部署。

## 运行入口

整套业务现在使用一个入口 `bash tools/pick_stack/run_once.sh`，由`Demo.run(RealIO)`连续调度。显式阶段为`STARTUP_PRECHECK → STARTUP_OPEN → HOME_START → PRECHECK → SCAN → 抓取/叠放循环 → RETURN_HOME`。启动张爪使用独立的`startup_open_raw=1000`，允许释放原有夹持物，核对真实张开反馈后才实测回Home。启动设备前检与完整抓放标定前检分开，后者缺项不会跳过启动张爪和Home。内部扫描器跳过重复的起始Home，但仍保留扫完回Home；独立扫描器默认行为不变。维护用的参考恢复/受控使能不混入正常业务循环，也不会在运行途中自动重启驱动。

```bash
# 实体业务启动会先张爪、回Home；当前抓放配置缺测时随后在PRECHECK退出。
bash tools/pick_stack/run_once.sh \
  --execute --authorization '本聊天已授权单枚整套尝试' \
  --output runs/pick_stack_NEW_FULL
```

输出`workflow.json`记录整套顺序和唯一调度器，`run-result.json`记录完成或具体阻塞阶段。默认三枚；真实接触/支承数据齐备后由代码逐段执行，不需要代理逐个调用机械臂或夹爪接口。首轮的人工承托/放稳确认仍按下述协议处理，不能把它称为已验收的无人值守堆叠。

从仓库根目录运行；每次使用新输出目录。普通审查/演练不导入ROS。

```bash
python3 tools/pick_stack/demo.py \
  --scan-result runs/scan_20cm_20261103_demo02/scan-result.json \
  --output runs/pick_stack_NEW_REHEARSAL --rehearse

source /opt/ros/humble/setup.bash
source runs/project_release_20261102_01/install/setup.bash
export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1
# 只读前检：不发机械臂或夹爪目标。
PYTHONDONTWRITEBYTECODE=1 python3 tools/pick_stack/demo.py \
  --output runs/pick_stack_NEW_PREFLIGHT --preflight
```

`--plan-current`配合历史`--scan-result`保留当前起点的独立端点检查，全为plan_only=true，不表示完整连续抓放路径已通过。

真实运行入口为 `--execute --authorization '本轮明确授权记录' --output runs/NEW`。它始终重新扫描，拒绝同时传入历史`--scan-result`；默认不执行。当前配置仍缺完整抓放量测；设备前检通过后会执行启动张爪及Home，然后在抓放前检中退出。`--preflight`、审查、演练及只规划模式保持无实体执行，创建同名检查节点不会自动触发启动动作。

本轮启动顺序和三颗行为验证见[调度变更记录](../../docs/validation/xiangqi-startup-stack-three-20261104.md)。

## 启动与业务方法

`Demo.prepare_startup()`编排设备前检、启动张爪与起始Home；`Demo.return_home()`统一起始和结束Home；`Demo.scan_workspace()`调用扫描并核对结束Home/停稳。`Demo.grasp_piece()`完成单枚重观、对齐、下降、夹持与抬升；`Demo.place_piece()`完成转移、承托、释放、撤离及放稳核对；`Demo.stack_pieces()`循环编排并累积层数。`RealIO`负责现有Action及反馈，业务方法不自建驱动。

已有夹持物也按本次启动规则先松开。反馈失效、故障、未初始化或未停稳仍会退出；Home先只规划，若当前位置（例如低位）不满足普通Home几何规则，则停止，不自动强行回Home或追加未经标定的撤离路径。启动、只规划和故障退出互相区分。

## 校准与接触/持物边界

台面base_z=-.010 m，棋厚.013 m，棋盘厚.014 m。裸桌第k层理论顶面 `-.010+.013*k`，1层.003 m，8层.094 m，绝不加棋盘厚。固定XY来自原日常Home模型FK投影，不用当前停靠姿态重定义Home。抓取处是否有棋盘不作假设，使用新测棋面与实测工具偏移。

多次用户量测已集中保存在[高度量测记录](../../docs/validation/xiangqi-height-measurements-20261104.md)：张爪约130/53/30mm、完全闭爪约5mm及红卒单点候选高度。开合状态与参照假设分别保留。

“TCP偏移”是软件假定的抓取点与两指实际夹住棋子的位置之间的距离。当前URDF的`grasp_link_offset=.20m`在description文档里明确是“旧抓取参考点，待定义/标定”，不是现场量测。不能直接把20cm填成实测参数。标定需要确定真实夹持中心和最低指尖相对于安装面的位置，再换算成代码参数。

`demo.current.json`的measured字段必须以现场量测填写，包括棋面→名义TCP、观察位、最低点/接触包络、半径、开合raw、支承/退出与保持释放证据、堆顶观察位。raw不是毫米、force_percent不是牛顿，现场力限20..30%。stack_stability_observation_pose为当前固定朝向下的名义TCP xyz，必须验证该位置能看到堆顶。

`commissioning.review.example.json`是**未批准的空模板**。真实review文件必须绑定完整配置的`calibration_sha256`、扫描配置的`source_sha256`、最大批次和reviewer，并引用接触策略、持物/堆叠审查资料的文件路径及SHA256。计算配置指纹可调用`demo.calibration_fingerprint(config)`；review路径本身不参与指纹。没有报告时不能用布尔ready或历史扫描授权代替。

原arm守卫仍检查固定工具、关节、IK、扭转和地面净空。**它没有自动加入被持棋子和增长中的堆叠碰撞体，本次也没有改arm规划器实现该能力。** 因此真实阶段为受监督demo：需要上述独立实物/包络审查，每次持物移动和每次下降还须人工确认具体SEGMENT请求。只规划返回的轨迹是审查资料；执行Action会从真实起点重新规划，不承诺逐点复用预览轨迹。人工须审查该操作的可达路径包络，不能只根据一条预览认定所有路径安全。不能由回复topic绕过arm内部拒绝。

`geometry_bounds.py`保留原模型下的必要端点计算：p1候选朝向要求名义TCP至少约.010823 m；直接把名义TCP设为首层棋面.003 m不满足固定包络地面条件。这不是实际标定抓放高度。

## 人工确认协议

### 对位观察模式（不接触、不夹取）

```bash
bash tools/pick_stack/run_once.sh --observe-alignment \
  --authorization '本次明确授权对位观察，不接触不夹取' \
  --scan-result runs/home_recovery_scan_20261104_01/scan/scan-result.json \
  --output runs/alignment_NEW
```

这是标定前的有限观察任务：选孤立种子→去已经受检的扫描观察点→至少两秒独立新帧重关联→受检移动到棋子XY对齐、名义TCP高于棋面100mm且不低于已验证观察点的姿态→实测停稳并保持，供人观察实际夹指间隙。只复用机械臂Action，不发送夹爪命令，不初始化，不自动下降或返回Home，不把模型点称作实测TCP，也不自动写入标定数据。失败只取消自有任务并核对停稳。

本次黑卒对位证据：`runs/pick_stack_alignment_20261104_01/alignment-result.json`，2段实际PTP均完成；74个新鲜帧用于重关联，实际指尖间隙仍待现场观察。

请求含task_id、request_id、issued_ns、kind、layer及具体资料。kind包括WORKSPACE、SEGMENT、SUPPORT和STABLE。SUPPORT在仍持物时确认已承托，STABLE在释放、撤离、两秒堆顶观测通过后确认放稳。每次最长等待60秒，期间继续核对停稳与持物；超时/拒绝即失败，不自动续跑。

人工读当前请求、完成实物核对后，向`/pick_stack/confirmation`回复以下结构；**示例ID不能复用**，observation须描述本次观察。没有自动确认节点。

```json
{"task_id":"从当前请求复制","request_id":"从当前请求复制","kind":"SUPPORT","layer":1,"decision":"confirmed","operator":"现场观察人","observation":"本次承托观察记录"}
```

错误或旧task/request/kind/layer不能推进流程；负面回复会中止。WORKSPACE包含独占控制的人工核对，本工具没有跨所有控制台/聊天的全局硬件租约。

## 新鲜度与故障

抓取使用两秒、至少20个独立新帧，并核对代际、源epoch、分类、完整目标、近邻、位移和重复性。预抓对齐后重观；实际抓取下降还会在规划/人工确认和最后停稳检查之后，再取新窗口，发送目标前核对最后帧≤0.5秒。unknown、近邻、位移或遮挡时有限重观或退出。已放源位置和码放区从后续种子选择排除，track_id不当跨视角物理ID，9个假设不当9枚。

机械臂分别核对Action结果、有效实测TCP、位置/朝向与连续停稳；正式Home还核对日常关节。夹爪核对真实新GRASPED、指令后变化、释放到位和反馈时效，拒绝旧held。已验证的Action最终样本进入单调反馈缓存，避免较旧topic消息倒灌。持物时持续检测掉落。

异常只取消本节点拥有的机械臂Action，再单独证明停稳；取消回执不等于停稳。夹爪不发取消，因为不能保证物理停止；只等待已提交结果并记录未知状态。故障处理不自动Home、开爪、初始化、重连、清错、使能、恢复参考或重放；正常执行入口的首次张爪/Home属于显式启动业务。层数只在承托、释放、撤离、堆顶观测与人工放稳全部通过后递增。

隔离ROS接口验证与本次限制见[接入验证记录](../../docs/validation/xiangqi-pick-stack-ros-integration-20261104.md)；先前静态几何与断链记录见[2026-11-03记录](../../docs/validation/xiangqi-pick-stack-20261103.md)。
