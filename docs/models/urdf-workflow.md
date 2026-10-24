# AI 辅助 URDF 装配与验证路线

版本：2026-10-24。由 Nova5、AG95 与两代相机支架的实际组装/排错整理。

## 先确定原则

把工作固化为 **输入版本 → 几何清点 → 安装基准 → 参数化装配 → 数值与图像检查 → 碰撞与场景 → 实物验收**。每一步有明确产物，后续 AI 根据同样的文件、命令和证据继续，而不是重新凭文件名和外观猜测。

AI 负责解释装配意图、修改参数和组织证据；网格解析器、正运动学与碰撞库负责计算。工具无法自动证明“给的文件就是实物版本”，也无法从任意 STL 可靠恢复安装语义。人需要确认零件版本和安装面，并提供实测基准。

## 七步流程与出口条件

| 步骤 | 做什么 | 必须留下什么 | 什么时候可以进入下一步 |
| --- | --- | --- | --- |
| 1. 明确输入和用途 | 确认硬件型号/版本、原装或定制手指、相机和支架；区分展示、避障规划、动力学仿真 | 原始文件副本、SHA-256、单位来源、来源日期、硬件版本说明 | 用户/CAD 能确认输入对应当前实物；不确定项标为待确认 |
| 2. 清点几何 | 读取尺寸、原始坐标范围、分量、三角形数、闭合性；画带分量编号的三视图 | `mesh.json`、`mesh.png`；分量编号和保留/排除理由 | 明确是单件、复合装配、安装件还是完整夹爪；不靠文件名下结论 |
| 3. 定义安装基准 | 标记法兰面、夹持槽定位面、关节轴、TCP/相机参考系；写清坐标变换方向 | 坐标系示意、4×4 变换、单位、每个偏移的来源 | 明确哪个网格自带底座、定位凸台和垫片；避免重复叠加厚度 |
| 4. 参数化装配 | 修改 Xacro 源入口和参数，必要时从原网格生成新派生件；visual/collision 分开 | Xacro、派生 STL、来源清单、生成的 URDF、Git diff | 单位和路径可解析，关节树合法，原件可追溯，没有重复静态/活动几何 |
| 5. 静态与运动学检查 | 独立解析，检查关节轴/限位/mimic、TCP 方向、多个开度；分别渲染 visual 和 collision | `check_urdf` 日志、FK/惯性报告、全开/半开/近闭合三视图 | 数值和图片都符合预期；图片由人或具备看图能力的模型复核 |
| 6. 碰撞与任务场景 | 指定需要检查的 link 对，扫关节；再用 SRDF/ACM 和完整任务场景检查整机与轨迹 | 原始碰撞对、采样范围/步数、历史失败姿态、允许接触理由、规划日志 | 预期配合接触与异常碰撞被区分；未全局关闭危险碰撞对 |
| 7. 实物对照与回归 | 测量安装偏移/开口/TCP，核对控制器反馈关节，验证任务轨迹并记录误差 | 实测表、标定结果、同姿态虚实对照、固定回归案例 | 达到这个任务事先约定的误差/间隙标准；以后改模型重跑同一组案例 |

展示用途可以先交付前五步的结果。用于真实避障/执行，需要补齐后两步；需要动力学仿真时，还必须有可信的质量、质心、惯性及闭环/接触模型。没有所有任务通用的一条“URDF 已正确”判定。

## 本次经验如何变成规则

| 本次现象 | 根因或教训 | 固化规则 |
| --- | --- | --- |
| 把 `new_jiazhua1.STL` 当支架迁入 | 它实际含完整固定夹爪，叠在可动 AG95 上；早期只确认来源/解析，没有核实内容 | 首次使用每个网格都做分量清点和独立三视图，来源相同不代表建模正确 |
| 后来用户更正相机版本 | 文件本身可解析，仍可能对应旧实物 | 输入版本确认是第一关，AI 不能单凭网格判定与现场一致 |
| 原关节上限 1.10 rad 能解析 | 当前指垫网格已经互相穿入 | 关节限位要与实际网格运动联合检查，不能只核对 XML 有无 limit |
| 为避免实物自碰撞增加大盒 | 旧误差来源尚未查明；内侧每边多 10 mm，又损失了 20 mm 夹口 | 先查坐标/单位/反馈姿态/ACM/漏建零件，再按误差来源细设余量，区分接触面与非接触面 |
| 新支架和 AG95 base 外形重复 | 两份数据都包含夹持底座；额外 8.5 mm 不能再照搬 | 画出安装基准，明确每个物理零件只由哪一份几何负责 |
| 在线查看器空白 | 一次是 WebGL 被禁用；另一次是只导入 URDF 没有 STL | 先分清浏览器渲染、资源路径、几何装配三类问题；保留不依赖 WebGL 的 PNG 路线 |

本例的 0.94 rad、零额外安装 Z、200 mm 抓取参考点，都是本项目条件下的值，**不得复制为其他机器人的默认正确值**。

## view_image 在流程中是什么

这次的链路是：

`STL/URDF → Trimesh/FK 的数值与几何 → Matplotlib 三视图 PNG → view_image 读取 PNG → 人/AI 复核`。

`view_image` 读取图片，不直接解析 STL，也不执行碰撞求解。它帮助识别重复零件、相机版本、前后/左右方向和可见错位；精确尺寸、轴变换、夹口间距来自文件计算。碰撞是否发生不能仅凭投影图判断。

没有视觉能力的模型仍可运行命令、读 JSON、定位异常，再让人打开 PNG 完成图像复核。这种情况下应记录“待人工看图”，不能声称已经看过。即使有视觉能力，图像复核也不等于实物精度证明。

三视图使用 Matplotlib 的离线栅格绘制，不需要浏览器 GPU、WebGL 或显示服务器；没有恢复此前移除的本地网页查看器。

## 工具选择：核心栈固定下来

| 职责 | 当前选定工具 | 使用方式与边界 |
| --- | --- | --- |
| 源模型修改/参数化 | Xacro + Git | Xacro 是源文件，展开 URDF 是生成物；AI 修改最小参数/模块，保留 diff 和版本 |
| 网格参数与分量 | Trimesh + `inspect_mesh.py` | 尺寸、分量、闭合性、候选平面和来源；不自动认定安装面/材料/单位 |
| 派生网格 | `derive_mesh.py` | 按显式 SHA、单位、分量列表和刚体变换生成新文件，不覆盖原件，不自动缩放拟合 |
| 数值/FK 与无 GPU 图片 | `inspect_urdf.py` + Matplotlib | 纯 URDF、关节姿态、各 link 变换、visual/collision 分视图、惯性基本约束；支持 fixed/revolute/continuous/prismatic |
| 正式 URDF 解析 | urdfdom `check_urdf` | 独立解析器，作为文件结构检查；不证明安装尺寸或实物碰撞 |
| 离线指定碰撞对 | python-fcl，接入 `inspect_urdf.py` | 本地关节扫描；原生 box/sphere/cylinder，网格用三角 BVH；无依赖就明确 NOT_RUN |
| 整机规划与接触规则 | MoveIt 2 + SRDF/ACM | 定义规划组、碰撞检查对和任务场景，再检查离散/连续轨迹；本项目尚未完成这一层 |
| 人工快速查看/小修改 | 现有 URDF Studio；ROS 环境可用 RViz | 支持可视化操作；导出后必须回到源文件/diff/回归检查流程 |

依据：Trimesh 提供网格边界、分量分离及拓扑/体积属性；FCL 提供碰撞、距离和连续碰撞功能；MoveIt Setup Assistant 提供 SRDF 和自碰撞矩阵配置。[Trimesh](https://trimesh.org/)、[FCL](https://github.com/flexible-collision-library/fcl)、[MoveIt Setup Assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)。注意本次 CLI 实现的是离散指定对扫描，**没有实现 FCL 的全部能力或连续碰撞检查**。

URDF Studio 是当前人工查看和编辑入口，无需再装一个查看器才能做这套检查。[官方仓库](https://github.com/OpenLegged/URDF-Studio)

按需候选：需要长期可视化构建复杂机器人时，可以单独评估 Blender 的 Phobos；已有 Onshape CAD 流程时，可以评估 onshape-to-robot 的 CAD 导出。它们目前不是核心依赖，也未在本项目安装或做兼容性验收。[Phobos](https://github.com/dfki-ric/phobos)、[onshape-to-robot](https://github.com/Rhoban/onshape-to-robot)

## 本次补齐的命令入口

原有 `generate_models.py`、`check_models.py`、`extract_camera_mount.py`、`export_browser_model.py` 继续作为本项目生成/回归工具。新入口对输入文件参数化，不要求换机器人就重写一套脚本。

### 1. STL 清点

```bash
python tools/description/inspect_mesh.py path/to/source.stl \
  --units mm --json runs/inspection/mesh.json --png runs/inspection/mesh.png
```

输出 SHA、原始尺寸和 SI 尺寸、分量编号/包围盒、三角形数、闭合性/法向一致性、表面积、包围盒体积、条件满足时的网格体积、候选轴对齐平面和三视图。省略 `--units` 时标为 unknown，SI 尺寸为空，不猜毫米/米。

候选平面不是自动识别的法兰面；连通分量不是机械零件名称；AABB 体积不是实体体积；重叠零件的体积相加不是几何并集。工具不凭缺少密度的 STL 给出真实质量。

### 2. 派生件

```bash
python tools/description/derive_mesh.py path/to/source.stl \
  --spec tools/description/examples/camera-v2-derive.json \
  --output runs/inspection/new-part.stl
```

spec 必须明确 `input_sha256`、`units`、`components`（all 或编号列表）及 `transform_m`。该变换在将源坐标统一为米后应用。工具拒绝源版本不符、反射、拟合缩放，以及覆盖原件/已有产物；旁边写入 provenance JSON。示例 spec 只适用于本项目的新相机文件，其他网格需要重新确定。

### 3. URDF 快照、三视图、指定碰撞对

```bash
python tools/description/inspect_urdf.py \
  src/chess_robot_description/urdf/nova5_with_ag95.urdf \
  --package-root src --joint left_outer_knuckle_joint=0.47 \
  --frame Link6 --subtree Link6 --show-frame grasp_link \
  --json runs/inspection/urdf.json \
  --visual-png runs/inspection/visual.png \
  --collision-png runs/inspection/collision.png \
  --pair left_finger_pad right_finger_pad \
  --sweep left_outer_knuckle_joint 0 0.94 101 --require-clear
```

可重复 `--package-root`、`--joint`、`--show-frame`、`--pair`。也可用 `--positions case.json` 传入独立关节位置；mimic 从主关节推导。不按文件名兜底猜路径，重复包路径导致歧义时直接报错。

不带 `--pair` 则碰撞是 NOT_RUN；带 `--pair` 而缺少 FCL 时退出 2 并明确 NOT_RUN。退出码 0 表示报告成功生成，**不是全模型验证通过**；`--require-clear` 在指定采样对有接触时退出 1；输入/依赖错误退出 2。指定对可以是有意配合接触，需根据任务解释，不能见到接触就修改全局忽略规则。

网格 BVH 主要检查表面相交，完全包裹但表面不交的实体情况仍需实体包含/凸体方法补充。离散采样可能遗漏样本之间的碰撞，真实轨迹还需连续或足够分辨率的规划场景验证。任意 CAD 孔轴/非轴对齐安装面自动识别、通用闭环动力学和自动 TCP 标定不在这三个脚本的能力范围内。

### 4. 依赖与回归

在独立环境安装，保留 ROS 环境：

```bash
python3 -m venv runs/urdf_tools_env
runs/urdf_tools_env/bin/python -m pip install -r tools/description/requirements-geometry.txt
# 需要 Python 碰撞绑定时才安装；若平台无可用包，保持 NOT_RUN，改用 MoveIt/C++ FCL 验证。
runs/urdf_tools_env/bin/python -m pip install -r tools/description/requirements-collision.txt
runs/urdf_tools_env/bin/python tools/description/test_workbench.py
```

Windows 使用相应环境的 `Scripts/python.exe`。核心工具测试覆盖：单位未知不冒充米、ASCII STL、源文件不变、显式刚体派生、拒绝反射/错误 SHA/覆盖、prismatic 正运动学、FCL 原生形状正反例或缺依赖 NOT_RUN、拒绝不支持关节与缺失网格。

本次实际验证发现 AGX 系统 NumPy 1.21.5 无法导入 Trimesh 5.2.0。已在 `runs/urdf_tools_env` 独立环境中安装 NumPy 1.24.4 与 Trimesh 5.2.0，复用系统 SciPy 1.8.0 / Matplotlib 3.5.1，未修改 ROS 的系统 NumPy。网格清点、派生件、三视图和六项回归在 AGX 通过。

AGX 当前没有 Python FCL 绑定；已实际验证请求碰撞时返回 NOT_RUN 和退出码 2。本地 Windows 分析环境有 python-fcl 0.7.0.11，原生形状碰撞正反例与当前指垫扫描已通过。AGX 可直接调用 `runs/urdf_tools_env/bin/python tools/description/inspect_mesh.py ...` 等核心命令；需要碰撞时选择已有 FCL 环境或后续 MoveIt/C++ 路线。

不能把某个开发机的依赖可用性自动当作所有机器都可用。依赖文件给出安装约束，实际版本与运行结果以每次 JSON 内的 environment 和原始日志为准。

## 最小交付目录

```text
reference/                 原始文件、来源、版本、哈希
src/.../urdf/*.xacro       可编辑源，安装参数集中
src/.../urdf/*.urdf        生成物
src/.../meshes/            active visual / collision 派生件
docs/models/              安装基准、流程、使用方式
runs/<run-id>/             mesh/FK/碰撞 JSON、PNG、解析/构建日志
```

每次交付至少包括：输入清单、改动 diff、安装变换及依据、默认/边界姿态图、数值/碰撞报告、回归案例、仍未验证项。版本切换时生成新的 run-id，防止旧截图/旧 JSON 冒充新结果。

## 当前项目可以怎样表述状态

- 文件结构、来源、生成和模型资源检查已完成。
- 用户认为目前外观基本正确；新相机安装基准已按 CAD 配准。
- 指定手指/附件对已进行有限开度 FCL 检查；这不是整机工作空间或任务轨迹验收。
- 实机安装误差、控制器坐标映射、TCP/相机光学标定、完整 SRDF/ACM/棋盘场景、动力学质量参数尚未全部完成。

后续 AI 应读取 [接力说明](urdf-handoff.md)，按相同证据结构推进，不把上面的局部完成项提升为整体正确。
