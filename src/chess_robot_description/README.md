# Nova5 与 AG95 模型资产

ROS 2 description 包，提供 URDF、网格和共用的 Python 几何函数。Nova5 本体描述保持不变，不含控制器。

- `python/chess_robot_description/`：共用几何计算源码；安装后使用 `chess_robot_description` 包名导入。
- [urdf/README.txt](urdf/README.txt)：源文件、生成结果、编辑入口和按需生成命令。
- `urdf/nova5.urdf`：Nova5 原厂基线。
- `urdf/nova5_with_ag95.urdf.xacro`：组合模型入口。
- `urdf/nova5_with_ag95.urdf`：由入口生成的纯 URDF。
- `meshes/nova5/`：7 个原厂 STL。
- `meshes/tool_mount/camera_mount_assembly_m.stl`：用户提供的 20260117 v2 安装板、支架和新相机，已按法兰安装面配准到 Link6，单位为米。
- `meshes/tool_mount/camera_mount_assembly_m.json`：CAD 安装基准、坐标变换、7 个分量、来源与 SHA-256。
- `meshes/tool_mount/new_jiazhua1.STL`：原始复合网格，仅作来源对照，默认模型不引用；它包含完整的静态夹爪，不能再当独立支架叠加。

## 1 当前装配

本包负责 Nova5、安装件和夹爪的整机装配；AG95 本体定义与原始网格由 `dh_ag95_description` 提供，通过 Xacro 引用。

当前安装件采用用户提供的 20260117 v2 模型，包含安装板、支架、相机及夹持底座。组合模型因此省略 AG95 通用底座的重复显示和碰撞形状；单独查看夹爪时仍使用通用底座。

夹爪使用原有碰撞 STL，未附加指尖碰撞盒或 `safety_padding_link`。默认关节上限为 0.94 rad，对应当前网格约 0.648 mm 夹口；这是几何范围，不是实机驱动限位或标定结果。

## 2 构建与查看

```bash
cd /home/kuang/workspace/chess_robot
bash tools/build.sh --packages-select dh_ag95_description chess_robot_description
source tools/env.sh
```

需要导出模型供查看时，从项目根目录执行：

```bash
python3 tools/description/export_browser_model.py
```

在在线解析器中导入完整 `runs/browser_model.zip`，选择 `nova5_with_ag95.urdf`。只导入 URDF 不会带入 STL。

## 3 参数与安装基准

装配链为 `Link6 → camera_bracket_link → ag95_base_link`。用户 STL 已烘焙 CAD 配准变换和毫米缩放，在 URDF 中使用单位缩放、零 origin。

| Xacro 参数 | 默认值 | 状态 |
| --- | --- | --- |
| `gripper_mount_z` | 0.0 m | 新 CAD 的法兰安装面与底座基准重合；实机偏差待测 |
| `grasp_link_offset` | 0.20 m | 抓取参考点，待定义/标定 |
| `gripper_joint_upper_limit` | 0.94 rad | 几何检查范围，非硬件命令 |
| `include_ground` | false | 可选查看场景 |

实物采用原装指垫、带侧面相机的嵌槽式支架。新 CAD 的法兰圆心/安装面为原始坐标 `[46.5,92,107] mm`；旋转 180°（Y 轴）并移至原点后，共有底座平面与 AG95 基准的 Z=-3.5、0、5、14.7 mm 对应。5 mm 是安装板实体厚度，3.5 mm 是定位凸台，14.7 mm 是夹持槽外形上界，均不应额外叠加为整体安装偏移。新夹持槽面 X=±40.55 mm，与通用底座 ±40.5 mm 接近。因此不叠加额外 8.5 mm 安装偏移，保留 `gripper_mount_z` 供实测后微调。

新固定网格承担夹持底座和相机外形，组合模型的 `ag95_base_link` 仍保留坐标系、关节和原惯性，但不再重复显示/碰撞。尚未测量新相机/支架质量或创建相机光学标定坐标系。

默认 `grasp_link` 的局部 +X 指向夹爪伸出方向，其 +Z 不是接近轴；抓取代码需要使用一致的轴约定。

## 4 验证边界

本体的六个关节 effort/velocity 为原厂 0 占位值。尚无完整 SRDF、有效运动限值或实机碰撞误差验收。删除旧包络不代表碰撞检查已关闭，也不代表历史自碰撞原因已消除；细化后的模型仍需验证安装基准、误差余量和实际碰撞姿态。

查看和参数覆盖见 [模型查看说明](../../docs/models/ag95-browser.md)。生成条件与入口见 [URDF 提示](urdf/README.txt)。CAD 配准不等于实物安装/TCP 标定；按本次改动选择相关检查，不把构建或查看当成完整验收。
