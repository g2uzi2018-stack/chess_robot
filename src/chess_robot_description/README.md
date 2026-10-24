# Nova5 与 AG95 模型资产

数据型 ROS 2 description 包，提供 URDF 和网格。Nova5 本体描述保持不变，不含控制器。

- `urdf/nova5.urdf`：Nova5 原厂基线。
- `urdf/nova5_with_ag95.urdf.xacro`：组合模型入口。
- `urdf/nova5_with_ag95.urdf`：由入口生成的纯 URDF。
- `meshes/nova5/`：7 个原厂 STL。
- `meshes/tool_mount/camera_mount_assembly_m.stl`：用户提供的 20260117 v2 安装板、支架和新相机，已按法兰安装面配准到 Link6，单位为米。
- `meshes/tool_mount/camera_mount_assembly_m.json`：CAD 安装基准、坐标变换、7 个分量、来源与 SHA-256。
- `meshes/tool_mount/new_jiazhua1.STL`：原始复合网格，仅作来源对照，默认模型不引用；它包含完整的静态夹爪，不能再当独立支架叠加。

## 本轮修正

2026-10-24 移除手指/指垫共四个附加碰撞盒和蓝色 `safety_padding_link`；保留 AG95 原碰撞 STL。固定复合夹爪改为仅保留后侧相机和安装件，活动 AG95 只出现一次。

同日用户更正相机版本，提供 `机械臂+夹爪_20260117(2).STL`。检查确认它只包含固定安装件与新相机，并非机械臂/活动夹爪。当前已用其完整 7 个分量替换旧相机版本。新文件含夹爪的夹持底座，因此组合模型省略通用 AG95 base 的重复 visual/collision；独立夹爪模型仍保留通用底座。

主动关节的默认模型上限改为 0.94 rad，对应当前网格约 0.648 mm 夹口。它是防止模型自穿入的几何限值，不是实机驱动限位或标定。

## 构建与查看

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dh_ag95_description chess_robot_description
source install/setup.bash
python3 tools/description/check_models.py
python3 tools/description/export_browser_model.py
```

在在线解析器中导入完整 `runs/browser_model.zip`，选择 `nova5_with_ag95.urdf`。只导入 URDF 不会带入 STL。

## 参数与安装基准

装配链为 `Link6 → camera_bracket_link → ag95_base_link`。用户 STL 已烘焙 CAD 配准变换和毫米缩放，在 URDF 中使用单位缩放、零 origin。

| Xacro 参数 | 默认值 | 状态 |
| --- | --- | --- |
| `gripper_mount_z` | 0.0 m | 新 CAD 的法兰安装面与底座基准重合；实机偏差待测 |
| `grasp_link_offset` | 0.20 m | 旧抓取参考点，待定义/标定 |
| `gripper_joint_upper_limit` | 0.94 rad | 几何检查范围，非硬件命令 |
| `include_ground` | false | 可选查看场景 |

实物采用原装指垫、带侧面相机的嵌槽式支架。新 CAD 的法兰圆心/安装面为原始坐标 `[46.5,92,107] mm`；旋转 180°（Y 轴）并移至原点后，共有底座平面与 AG95 基准的 Z=-3.5、0、5、14.7 mm 对应。5 mm 是安装板实体厚度，3.5 mm 是定位凸台，14.7 mm 是夹持槽外形上界，均不应额外叠加为整体安装偏移。新夹持槽面 X=±40.55 mm，与通用底座 ±40.5 mm 接近。因此去掉旧的额外 8.5 mm，保留 `gripper_mount_z` 供实测后微调。

新固定网格承担夹持底座和相机外形，组合模型的 `ag95_base_link` 仍保留坐标系、关节和原惯性，但不再重复显示/碰撞。尚未测量新相机/支架质量或创建相机光学标定坐标系。

默认 `grasp_link` 的局部 +X 指向夹爪伸出方向，其 +Z 不是接近轴；抓取代码需要使用一致的轴约定。

## 验证边界

本体的六个关节 effort/velocity 为原厂 0 占位值。尚无完整 SRDF、有效运动限值或实机碰撞误差验收。删除旧包络不代表碰撞检查已关闭，也不代表历史自碰撞原因已消除；细化后的模型仍需验证安装基准、误差余量和实际碰撞姿态。

重新生成、参数覆盖和本轮检查见 `../../docs/models/ag95-browser.md` 和 `../../docs/validation/camera-v2-2026-10-24.md`。CAD 配准不等于实物安装/TCP 标定。
