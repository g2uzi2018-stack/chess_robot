# Nova5 与 AG95 模型资产

数据型 ROS 2 description 包，安装 URDF 和网格；原厂 Nova5 本体文件保持不变。

- `urdf/nova5.urdf`：原厂纯 URDF，仅将网格 URI 的包名改为 `chess_robot_description`。
- `urdf/nova5_with_ag95.urdf.xacro`：组合入口，引用相邻的 `dh_ag95_description` 包。
- `urdf/nova5_with_ag95.urdf`：生成的组合纯 URDF，供浏览器或 RViz 加载。
- `meshes/nova5/`：与该 URDF 对应的 7 个原厂 STL 文件。
- `meshes/tool_mount/new_jiazhua1.STL`：旧装配实际使用的支架网格，字节不变。
- 原始 URDF 对照件、来源提交和文件校验值见仓库的 `reference/dobot/`。

## 使用边界

`nova5.urdf` 仍只含本体；`nova5_with_ag95.urdf` 包含 AG95、支架和旧碰撞补丁。
组合模型不含棋盘、桌面或独立的相机光学坐标系。
原厂导出文件的六个转动关节 `velocity`、`effort` 都为 0，本次按原样保留。当前模型可作为可视化和结构建模基础，尚未配置可执行的运动限值。
保持原始关节名称、坐标变换、惯性、几何和位置限位，尚未根据当前实机做精度与方向验收。

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dh_ag95_description chess_robot_description
source install/setup.bash
ros2 pkg prefix --share chess_robot_description
```

## 装配关系

`Link6 → camera_bracket_link → ag95_base_link → ag95_body / fingers`

- `Link6 → camera_bracket_link`：零平移、零旋转。
- 支架网格自身 origin：`xyz="0.09419 -0.10218 0.18445"`、`rpy="0 3.14159 0"`，缩放 `0.001`。
- 支架 → 夹爪基座：`xyz="0 0 0.0085"`、零旋转。
- 夹爪基座 → `grasp_link`：`xyz="0 0 0.20"`、`rpy="pi/2 -pi/2 0"`。
- 因此 `Link6 → grasp_link` 的位置为 `[0, 0, 0.2085] m`，方向包含上述固定旋转。

这些是从旧装配文件继承的参数，不是本次实测或标定结果。
`camera_bracket_link` 保留历史名称和原网格，不表示已单独建立相机光学模型。
默认保留旧蓝色防碰撞包络和手指处的额外碰撞盒；蓝色薄片是人工包络，不是夹爪零件。
Xacro 参数 `include_legacy_padding:=false` 只关闭蓝色薄片，原宏内的手指碰撞盒仍保留。
`include_ground:=true` 可恢复旧装配文件中的地面，默认不包含地面。

纯 URDF 已随仓库提供，查看无需先展开 Xacro。浏览器步骤、重新生成和检查命令见
`../../docs/models/ag95-browser.md`。夹爪关节限值、惯性和碰撞补丁保留旧值，尚未实机验收。
来源与原装配对照件见 `../../reference/ag95/`。
