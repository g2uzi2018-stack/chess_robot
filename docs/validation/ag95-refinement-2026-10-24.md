# AG95 第一轮几何修正（2026-10-24）

## 结果与范围

用户说明旧包络曾用于补偿“模型没碰、实物碰到了机械臂”的误差，并授权移除、继续细化模型。当前已移除旧额外包络、分离侧面附件、消除固定重复夹爪，并增加几何回归检查。原碰撞 STL 仍然参与碰撞检查，没有增加全局忽略规则或禁用夹爪与机械臂的碰撞。

这一轮不代表历史自碰撞原因已消除；安装基准/TCP 和实际误差余量仍需校准。

## 具体变更

- 删除 finger、finger_pad 宏中的两个额外 collision box，左右实例共四个。
- 删除组合模型的 `safety_padding_link`、固定关节及蓝色薄片外观/碰撞。
- 将原始 AG95 宏另存为 `reference/ag95/dh_ag95_macro.original.xacro`，保留原字节和来源 SHA-256。
- 12 个夹爪 visual/collision STL、7 个 Nova5 STL 和原复合 STL 不变。
- 原复合 STL 中只保留经前/侧视图检查的后侧相机/安装件：155 个连通分量中选取 102 个完整分量，38,290 个三角形，无三角形裁切。旧 origin 与缩放烘焙为 Link6 坐标，输出 STL 单位为米。
- 排除的夹爪本体、底座、连杆和手指由活动 AG95 描述提供；原复合 STL 留作对照，默认 visual/collision 不再引用它。安装板与夹爪底座的实际归属还需独立支架 STL 确认。
- 默认模型闭合上限从 1.10 改为 0.94 rad；当前网格指间距约 0.64787 mm，避免原 1.10 rad 的实体自穿入。此值不是硬件限位标定。
- 安装 Z、抓取参考点 Z、关节上限变成 Xacro 参数；保留未标定的 8.5 mm 和 200 mm 默认值。

## 验证

- 标准库 `check_models.py`：来源快照、原厂本体不变、关节树、mimic/限值、网格引用、无旧包络、新相机网格来源及上下限夹口几何通过。
- Xacro 重新生成并进行 `--check`；派生网格 `extract_camera_mount.py --check` 逐字节复现通过。
- 修正后组合模型：20 links、19 joints，20 个独立活动网格（计入三种模型的来源集合）。三个模型 70 条 mesh 引用通过。
- 本地离线 python-fcl 0.7.0.11 / trimesh 三角网格检查：在 0～0.94 rad 的 101 个样本中，两侧指垫 collision 未相撞。
- 回归对照：旧模型 q=0.825 rad 时两个附加指垫盒相撞；修正模型该角度不再出现该碰撞。
- 80 个固定随机种子的离线折臂姿态中，15 个样本仍检出工具/相机与机械臂的碰撞，说明真实碰撞几何并未被关闭。这些姿态是诊断样本，不是已规划或可执行轨迹。
- 默认零位与三个开度中仍有相邻件、固定安装件和夹爪闭环连接区域的原始接触报告。尚未用 SRDF/ACM 将它们分类，未静默屏蔽任何原始碰撞对。
- AGX 执行 ROS 2 Humble colcon、urdfdom 及安装后跨包 Xacro 检查；最终执行日志位于 `runs/20261024_ag95_refinement/`。

STL 表面碰撞和有限角度/姿态扫描不能证明所有姿态安全，也不能代替真实安装偏差验证。

## 安装结构待确认

实物为 AG95 原装指垫、带侧面相机的嵌槽式支架。夹爪后端两侧凹槽和螺丝与支架配合，支架中心孔连接机械臂末端法兰。

因此需要识别法兰贴合面、支架夹持槽/定位面和夹爪模型原点。支架总厚度不直接等于 `gripper_mount_z`；凹槽嵌入会影响安装变换。用户准备提供的独立支架 STL 可用于确认安装面和厚度。当前保留旧 Z=8.5 mm 并标记未标定，未因两份旧模型约 14 mm 的位置差异直接猜改。

下一轮：核对独立安装件/真实安装面 → 修改安装变换 → 对照实际夹口和指垫高度标定 TCP → 回放历史容易自碰撞的模型姿态并细设余量 → 接入有效关节限制与 SRDF 做棋盘场景规划。

## 重现命令

```bash
source /opt/ros/humble/setup.bash
python3 tools/description/generate_models.py --check
python3 tools/description/check_models.py
colcon build --packages-select dh_ag95_description chess_robot_description
source install/setup.bash
check_urdf src/chess_robot_description/urdf/nova5_with_ag95.urdf
xacro "$(ros2 pkg prefix --share chess_robot_description)/urdf/nova5_with_ag95.urdf.xacro" > runs/installed-assembly.urdf
check_urdf runs/installed-assembly.urdf
python3 tools/description/export_browser_model.py
```

重现后侧附件分离需要额外安装 numpy、trimesh；不是浏览器查看或标准库完整性检查的依赖：

```bash
python3 tools/description/extract_camera_mount.py --check
```

本轮未发出机械臂/夹爪运动、使能或限位写入指令。
