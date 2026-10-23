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
3. 如果解析器支持关节调节，可检查 `left_outer_knuckle_joint` 的 `0`、`0.55`、`1.10 rad` 三种状态；其余手指通过 mimic 联动。
4. 如果支持碰撞几何显示，可检查指尖、支架和旧碰撞包络。蓝色薄片是历史碰撞包络，不是实际零件。

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

- 旧支架网格是否对应当前安装件，夹爪是否仍为 AG95、手指是否改装。
- 支架至夹爪的 8.5 mm 安装偏移，抓取坐标系的 200 mm 偏移和方向。
- 开合行程与实际指间距的关系；原关节值以弧度表示，并非驱动开度值。
- 旧碰撞盒/包络是否适合象棋棋子的间距。

当前检查能确认模型可解析、网格可加载、关节链与旧装配一致；不能代替 TCP 标定、实机跟踪、
碰撞规划或动力学验证。机械臂本体仍保留原厂零速度/力矩占位值。
