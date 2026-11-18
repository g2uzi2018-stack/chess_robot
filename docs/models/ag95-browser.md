# 导出 Nova5 + AG95 供在线解析器查看

模型已装配到项目中。当前项目只提供模型资源和导出工具，不附带本地网页查看器或 HTTP 服务。

## 1 最简步骤

在 AGX 的项目根目录运行（只用 Python 标准库，不要求 ROS 环境）：

```bash
cd /home/kuang/workspace/chess_robot
python3 tools/description/export_browser_model.py
```

在你选择的在线 URDF 解析器中导入 `runs/browser_model` 整个文件夹，或按该解析器的要求上传 ZIP。

1. 选择 **nova5_with_ag95.urdf** 查看组合模型。
2. 选择 **ag95.urdf** 可只看夹爪。
3. 如果解析器支持关节调节，可检查 `left_outer_knuckle_joint` 的 `0`、`0.47`、`0.94 rad` 三种状态；其余手指通过 mimic 联动。
4. 如果支持碰撞几何显示，可检查原始碰撞 STL。旧附加指尖碰撞盒和蓝色薄片已移除，相机与安装件仍保留。

导出同时生成 `runs/browser_model.zip`，可复制到别的电脑解压查看。
导出包内网格均为相对路径，选择整个文件夹才能加载完整外形。
导出包只包含 URDF、STL、来源/许可证、说明与校验清单，不包含 HTML、JavaScript 或服务端程序。

也可以直接在查看器中选择本仓库的 `src` 文件夹，然后选择组合纯 URDF；
`chess_robot_description/package.xml` 用于解析 `package://` 路径。导出包则不依赖 ROS 包路径解析。

## 2 按需修改与生成

整机源入口为 `src/chess_robot_description/urdf/nova5_with_ag95.urdf.xacro`，
夹爪本体定义为 `src/chess_robot_description/urdf/ag95/dh_ag95_macro.xacro`。
`nova5.urdf` 是原厂本体输入；`nova5_with_ag95.urdf` 和 `ag95.urdf` 才是本工具生成的结果。

仅当输入变化影响展开结果、且需要使用或维护结果文件时，或本次需要的结果缺失/已确认过期时，执行：

```bash
cd /home/kuang/workspace/chess_robot
source /opt/ros/humble/setup.bash
python3 tools/description/generate_models.py
```

现有生成脚本一次处理 AG95 独立模型和整机模型两份 URDF，执行前确认范围。
不要手工修改这两份生成结果；需要浏览器查看包时再按第 1 节导出。
文档、说明性注释和无关代码调整不触发生成或 `--check`；仅替换同路径网格且 URDF 参数未变时，不重写 URDF。
正常构建或启动已有必要生成步骤时不重复执行。生成完成不自动串联全套检查；具体检查按本次改动选择。

文件职责与参数入口见模型包的 `urdf/README.txt` 和 `urdf/ag95/README.txt`。
这些工具处理离线模型，不负责机器人硬件控制。

可选场景（输出到运行目录，不覆盖默认纯 URDF）：

```bash
mkdir -p runs
source install/dev/local_setup.bash
xacro src/chess_robot_description/urdf/nova5_with_ag95.urdf.xacro \
  include_ground:=true > runs/nova5_with_ag95_ground.urdf
```

## 3 尚需对照实物的内容

- 夹爪为原装指垫，当前使用用户提供的 20260117 v2 新相机与嵌槽式支架 STL，替换了旧相机版本。
- 新 CAD 与法兰安装面配准后，额外安装偏移默认为 0；仍需核实实物是否与这份 CAD 相同。抓取参考点相对基座 200 mm，未实机标定。
- 开合行程与实际指间距的关系；原关节值以弧度表示，并非驱动开度值。
- 去掉旧包络后，应基于实际尺寸和标定误差设置有针对性的余量，并回放历史容易自碰撞的姿态。

默认 0.94 rad 是避免网格自穿入的模型范围，不是硬件限位。参数覆盖示例（仅生成文件）：

```bash
xacro src/chess_robot_description/urdf/nova5_with_ag95.urdf.xacro \
  gripper_mount_z:=0.0 grasp_link_offset:=0.20 \
  gripper_joint_upper_limit:=0.94 > runs/assembly_candidate.urdf
```

离线模型检查不能代替安装面/TCP 标定、实机跟踪、带 SRDF 的碰撞规划或动力学验证。已有检查资料见 `../../runs/records/validation/camera-v2-2026-10-24.md` 和 `../../runs/records/validation/ag95-refinement-2026-10-24.md`；其结论只适用于对应版本与检查范围。机械臂本体仍保留原厂零速度/力矩占位值。
