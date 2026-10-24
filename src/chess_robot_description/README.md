# Nova5 与 AG95 模型资产

数据型 ROS 2 description 包，提供 URDF 和网格。Nova5 本体描述保持不变，不含控制器。

- `urdf/nova5.urdf`：Nova5 原厂基线。
- `urdf/nova5_with_ag95.urdf.xacro`：组合模型入口。
- `urdf/nova5_with_ag95.urdf`：由入口生成的纯 URDF。
- `meshes/nova5/`：7 个原厂 STL。
- `meshes/tool_mount/camera_mount_assembly_m.stl`：从旧复合 STL 分离的侧面相机和安装件，已变换到 Link6，单位为米。
- `meshes/tool_mount/camera_mount_assembly_m.json`：分离方法、保留分量、来源与 SHA-256。
- `meshes/tool_mount/new_jiazhua1.STL`：原始复合网格，仅作来源对照，默认模型不引用；它包含完整的静态夹爪，不能再当独立支架叠加。

## 本轮修正

2026-10-24 移除手指/指垫共四个附加碰撞盒和蓝色 `safety_padding_link`；保留 AG95 原碰撞 STL。固定复合夹爪改为仅保留后侧相机和安装件，活动 AG95 只出现一次。

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

装配链为 `Link6 → camera_bracket_link → ag95_base_link`。相机分离网格已包含旧 origin 和毫米缩放，因此在新的 URDF 中用单位缩放、零 origin。

| Xacro 参数 | 默认值 | 状态 |
| --- | --- | --- |
| `gripper_mount_z` | 0.0085 m | 旧安装偏移，待实测 |
| `grasp_link_offset` | 0.20 m | 旧抓取参考点，待定义/标定 |
| `gripper_joint_upper_limit` | 0.94 rad | 几何检查范围，非硬件命令 |
| `include_ground` | false | 可选查看场景 |

用户确认实物采用原装指垫、带侧面相机的嵌槽式安装支架：夹爪后部通过两侧槽和螺丝固定，支架中心连接机械臂法兰。这种安装方式不能用支架外观总厚度直接推导 `gripper_mount_z`；必须核对法兰安装面、嵌槽位置和 AG95 模型原点。独立支架 STL 或实测数据到位前，保持旧偏移并明确标记待标定。

AG95 的 base 网格本身含夹持底座外形；其与独立安装板的归属、尺寸需要对照实物/CAD，不能再叠加整套复合夹爪来补偿。这里尚未创建相机光学标定坐标系。

默认 `grasp_link` 的局部 +X 指向夹爪伸出方向，其 +Z 不是接近轴；抓取代码需要使用一致的轴约定。

## 验证边界

本体的六个关节 effort/velocity 为原厂 0 占位值。尚无完整 SRDF、有效运动限值或实机碰撞误差验收。删除旧包络不代表碰撞检查已关闭，也不代表历史自碰撞原因已消除；细化后的模型仍需验证安装基准、误差余量和实际碰撞姿态。

重新生成、参数覆盖、FCL 检查结果见 `../../docs/models/ag95-browser.md` 和 `../../docs/validation/ag95-refinement-2026-10-24.md`。
