# 导出 Nova5 + AG95 供在线解析器查看

模型已装配到项目中。当前项目只提供模型资源和导出工具，不附带本地网页查看器或 HTTP 服务。

## 最简步骤

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
两个 package.xml 用于解析 `package://` 路径。导出包则不依赖 ROS 包路径解析。

## 修改与重新生成

ROS 2 Humble 环境中已有 Xacro 时：

```bash
source /opt/ros/humble/setup.bash
python3 tools/description/generate_models.py
python3 tools/description/generate_models.py --check
python3 tools/description/check_models.py
```

模型源文件为 `src/chess_robot_description/urdf/nova5_with_ag95.urdf.xacro`，
夹爪源宏为 `src/dh_ag95_description/urdf/dh_ag95_macro.xacro`。
不要直接修改生成的 `.urdf`；修改源文件后重新生成，再重新导出浏览器包。
这些 Python 工具只做离线模型处理，不属于机器人业务/控制代码。

可选场景（输出到运行目录，不覆盖默认纯 URDF）：

```bash
mkdir -p runs
source install/setup.bash
xacro src/chess_robot_description/urdf/nova5_with_ag95.urdf.xacro \
  include_ground:=true > runs/nova5_with_ag95_ground.urdf
```

## 尚需对照实物的内容

- 夹爪已确认是原装指垫、带相机和嵌槽式支架；侧附件已从旧整机复合网格中分离，独立安装板与夹爪底座的基准还需核对。
- 支架至夹爪的 8.5 mm 安装偏移，抓取坐标系的 200 mm 偏移和方向。
- 开合行程与实际指间距的关系；原关节值以弧度表示，并非驱动开度值。
- 去掉旧包络后，应基于实际尺寸和标定误差设置有针对性的余量，并回放历史容易自碰撞的姿态。

默认 0.94 rad 是避免网格自穿入的模型范围，不是硬件限位。参数覆盖示例（仅生成文件）：

```bash
xacro src/chess_robot_description/urdf/nova5_with_ag95.urdf.xacro \
  gripper_mount_z:=0.0085 grasp_link_offset:=0.20 \
  gripper_joint_upper_limit:=0.94 > runs/assembly_candidate.urdf
```

当前检查不能代替安装面/TCP 标定、实机跟踪、带 SRDF 的碰撞规划或动力学验证。本轮 FCL 离线结果见 `../validation/ag95-refinement-2026-10-24.md`。机械臂本体仍保留原厂零速度/力矩占位值。
