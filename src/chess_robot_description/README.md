# Nova5 模型资产

新建的数据型 ROS 2 description 包，仅安装 URDF 和网格资源，没有复制原项目的驱动、MoveIt 配置或启动文件。

- `urdf/nova5.urdf`：原厂纯 URDF，仅将网格 URI 的包名改为 `chess_robot_description`。
- `meshes/nova5/`：与该 URDF 对应的 7 个原厂 STL 文件。
- 原始 URDF 对照件、来源提交和文件校验值见仓库的 `reference/dobot/`。

## 使用边界

模型只包含机械臂本体，不包含 AG95、相机、棋盘或桌面；这些后续按实际安装建模。
原厂导出文件的六个转动关节 `velocity`、`effort` 都为 0，本次按原样保留。当前模型可作为可视化和结构建模基础，尚未配置可执行的运动限值。
保持原始关节名称、坐标变换、惯性、几何和位置限位，尚未根据当前实机做精度与方向验收。

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select chess_robot_description
source install/setup.bash
ros2 pkg prefix --share chess_robot_description
```
