# AG95 夹爪模型

来自 `g2uzi2018-stack/nova` 的 AG95 模型资源，保留 Apache-2.0 许可证。不含串口驱动或控制器。

- `meshes/visual/`、`meshes/collision/`：各 6 个原始 STL，字节未变。
- [urdf/README.txt](urdf/README.txt)：文件职责、编辑入口和按需生成命令。
- `urdf/dh_ag95_macro.xacro`：可复用的夹爪定义，保留上游惯性、关节轴和联动关系，未附加指尖碰撞盒。
- `urdf/ag95.urdf.xacro`：独立模型入口。
- `urdf/ag95.urdf`：生成的纯 URDF。
- `../../reference/ag95/dh_ag95_macro.original.xacro`：未修改的上游宏，包含历史包络，供追溯。
- `SOURCES.json`：来源仓库、采用的提交版本、原件文件指纹，以及本地修改和处理后文件的说明。

本包维护夹爪本体；`chess_robot_description` 引用本包，并负责安装件与整机连接。`meshes/visual/` 保留显示细节，`meshes/collision/` 使用已有的简化碰撞网格；两套文件按各自用途维护。

网格缩放为 0.001，长度单位为米。默认 `joint_upper_limit=0.94` rad，避免旧 1.10 rad 导致的指垫自穿入。当前网格下 0.94 rad 的指间距约 0.648 mm，仅用于几何模型验证；不要把它当成硬件开度命令或已经标定的行程。

独立入口支持 `gripper_joint_upper_limit` 与 `grasp_link_offset` 参数；组合入口还支持安装偏移。抓取参考点沿夹爪基座 +Z 偏移 0.20 m，尚待实物标定。

宏参数 `include_base_geometry` 默认为 true，独立夹爪使用通用底座外形。Nova5 组合入口设为 false，因为用户新相机支架 STL 已包含夹持底座；只省略重复 visual/collision，基座坐标系、关节和原惯性仍保留。

本模型用单主动关节加 mimic 表达无接触平行运动，并不完整模拟接触后的自适应包络抓取。惯性和驱动开度映射仍需与当前实物对照。

查看方式见 [模型查看说明](../../docs/models/ag95-browser.md)，编辑和生成条件见 [URDF 提示](urdf/README.txt)。仅查看模型或修改说明时不重新生成，也不运行全套模型检查。
