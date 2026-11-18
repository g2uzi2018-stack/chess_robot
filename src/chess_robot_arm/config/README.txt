机械臂配置提示

这些文件由 arm_node 或其核心库读取，属于本包直接依赖的配置。
构建后安装在 install/dev/chess_robot_arm/share/chess_robot_arm/config/。
不需要复制到 /etc。默认由 launch/arm.launch.py 加载；参数只在启动时读取。
启动和覆盖命令见 ../README.md。现场覆盖使用根目录 config/ 及现有部署入口。

参数文件
- arm.yaml：基础参数，默认只读、暂停、执行关闭。参数用途和单位直接写在文件旁。
- commissioning.yaml：真实设备小范围调试覆盖；限制相对日常 Home 的关节和工具移动范围。
- exploration.yaml：真实设备分档调试覆盖；每档有速度、运动范围和扭转上限。
  后两份不是独立完整配置，应在 arm.yaml、Home、历史参考之后加载。

Home 与历史参考
- home.yaml：根 config/arm/home.yaml 的逐字镜像，不能只改其中一份。
  home.joint_names：joint1～joint6，顺序固定。
  home.joint_positions_rad：日常返回目标，六个关节角，单位 rad。
  home.taught：已有采集目标的标记；false 时节点拒绝启动。
  修改入口是 tools/arm/arm_capture_home.sh；采集与应用见 docs/operations/arm-quickstart.md。
- 根 config/arm/twist_reference.yaml：线缆历史参考，构建时安装到本包 config/。
  reference.joint_positions_rad：历史几何基准，按 joint1～joint6，单位 rad。
  改日常 Home 不应改变此基准，也不会把已积累的线缆扭转清零。
- 根 Home 的文件哈希被扫描配置引用，包内镜像又要求逐字相同，因此字段说明放在此处。
  不为补注释重写这两份文件，也不以更新扫描配置中的哈希掩盖输入变化。

几何数据文件
- nova5_ag95_geometry.yaml：默认几何检查数据，包含机械臂和末端附件的 8 组凸包。
  凸包是包住模型顶点的几何外壳，供距离/碰撞检查使用，不是显示网格。
- nova5_ag95_table_contact.candidate.yaml：将末端外壳分成固定部分和左右手指的候选数据，
  共 10 组凸包；附带目标附近的手指接近桌面参数。默认 launch 不选择它；
  根 config/arm/table-contact-profile.candidate.yaml 和部分扫描配置会引用它。
- 两份几何文件都被扫描配置按原始文件哈希引用；直接补注释也会改变哈希，所以保留原文，
  在这里说明结构。不因阅读、整理说明或无关改动重新生成。

几何字段（由 src/geometry_guard.cpp 读取）
- joint_origins：六个关节零位变换；每个 4×4 矩阵按行展开，平移部分单位 m。
- axes：各关节自身坐标系中的旋转轴单位向量。
- lower / upper：joint1～joint6 的角度下界/上界，单位 rad，与 URDF 对应。
- tcp：工具工作点相对 Link6 法兰的 4×4 变换，按行展开；平移单位 m。
- hulls.link / name：所属杆件编号（0 为基座，1～6 对应 Link1～Link6）及名称。
- hulls.vertices：所属杆件坐标系中的顶点 XYZ，单位 m。
- hulls.faces：三角面的三个顶点索引，从 0 开始。
- hulls.role：候选文件中 finger_assembly 标识可参与目标附近桌面检查的手指组。
- ground_z_m：桌面在基座坐标系中的 Z，单位 m。
- clearance_m：普通活动部件相对桌面的最小间隙，单位 m。
- characteristic_length_m：计算运动接近奇异姿态的指标时，用于统一长度尺度，单位 m。
- table_contact.finger_clearance_m / target_radius_m：候选文件中的手指桌面间隙 /
  目标 XY 附近允许范围半径，单位 m；仅在显式 table_contact 竖直任务中使用。

几何来源与更新入口
- 基础数据的现有生成入口：tools/analysis/workspace_analysis.py。
  从项目根执行 python3 tools/analysis/workspace_analysis.py --project . --out runs/<新目录> --probe-only，
  输出为该目录的 model.yaml；该命令还做模型求解核对，不是日常启动步骤。
- 手指拆分入口：tools/description/generate_table_contact_geometry.py，
  参数为 --root <项目根> --output <候选YAML> --report <结果JSON>。
- 输入包括 description 包内的 URDF、网格和工具变换。仅在输入变化且确需更新模型时生成，
  核对后再决定替换及受影响扫描配置的处理；不能生成后自动覆盖现用数据。

ServoJ 参数来源
aheadtime 和 gain 是无单位系数，不按毫秒解释。参见厂家 V4 SDK 的 ServoJ 参数说明：
https://github.com/Dobot-Arm/TCP-IP-Python-V4/blob/main/dobot_api.py
