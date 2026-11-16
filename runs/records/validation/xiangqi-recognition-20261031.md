中国象棋识别、坐标联调与曝光修正

阶段说明（2026-10-31审校）：本文保留该阶段的试验结果、编号与配置，不能将其中“当前”视为最终部署。后续局部几何优化见 Notion 识别笔记；最终相机发布与预览见 [720p30验证](xiangqi-camera-30hz-20261031.md)。各阶段实时、离线回放和视频帧率的测量口径分别保留。

日期：2026-10-31（Asia/Taipei）

已完成的试验
已读取 Notion“中国象棋夹取”和手眼标定记录，连接 Jetson，加载用户指定 best.pt（SHA256 f4fce2c0217b2ce292820e45f1779654b4cf58da3d156aff6306d56ae6aabf7e）。相机彩色图与对齐深度实际均为 1280×720；请求30fps。本轮3秒观察到相机帧号/设备时间推进约29.99Hz，ROS订阅端收到约21.25Hz，不宣称全链路无丢帧30Hz。

曝光
原始自动曝光画面中，浅色棋子严重发白，黑色桌垫噪声明显。对比自动及2/5/10/16/25ms曝光、增益16后，选用手动25000微秒（25ms）、增益16。25ms对比样本整帧有效原生深度约85.36%，10ms约71.77%。现场源和安装YAML已同步，新增C++启动参数 exposure_mode（keep/auto/manual）、exposure_us（微秒）、gain，仍禁止运行时修改。当前帧元数据确认 exposure_us=25000、gain=16。D405彩色与深度共用曝光。环境光变化后应重新检查亮度。

识别与坐标
相机输入1280×720；模型推理imgsz=640，与训练配置一致。三帧均预测红兵（约97.6%）和黑象（95.5%～96.5%）。最初960输入曾错分并输出同框多类别，未据此声称模型具备通用准确率。实验保留同框候选类别并按IoU去重，不手工替换类别。
使用相同stamp的彩色图、米制深度、CameraInfo、FrameInfo，并匹配25ms内的实测关节/ArmState；检查连续静止至少0.6秒。分割内部深度经有效性检查和鲁棒平面拟合，反投影得到可见表面中心；由当前URDF FK的base_T_Link6乘以手眼候选flange_T_camera变换到base_link。计算的模型TCP与运行中实测模型TCP核对一致。没有复用旧标定板的base_T_board作为当前物体位置。

第三帧基座表面中心（毫米；仅实验估计）
红兵：X=-117.54，Y=-675.49，Z=-2.64
黑象：X=0.83，Y=-636.21，Z=-1.78
黑象轮廓触及画面底边，其中心估计可能偏移，需放入完整视野后再作定位验证。上述Z为机器人基座下坐标，负值不代表深度为负。没有测量实体定位绝对误差，三帧一致性不等于实体精度。

实时查看
局域网：http://192.168.30.26:8088，选择“检测叠加图”。已验证HTTP图像1280×720、序号持续增长；通常约4Hz，观察到延迟约0.18～0.29秒。原始采集stamp保留；反馈变化或帧龄超过0.5秒时抑制XYZ；无匹配停稳数据时显示等待状态。
输出 /pose/debug/image（实验叠加图）与 /pose/xiangqi/detections（带原始时间的实验JSON）；不发布生产TF。

范围
本轮是只读识别及表面位置试验，Python诊断预览尚非正式C++位姿节点；没有估计棋子字面朝向/完整六维位姿，也不是夹取TCP。没有发送机械臂、夹爪动作，没有更改手眼外参；机械臂与夹爪驱动进程保持运行。正式自动夹取仍需完整视野、定位误差验证、真实TCP与夹取流程。

Jetson证据与操作
主目录：/home/kuang/workspace/chess_robot/runs/xiangqi_recognition_20261031_02/
内容：曝光对比原图/深度、部署前二进制和两份YAML备份、同步采样及CDR、推理JSON与叠加图、只读实验代码、live-verification.json。
实时实验服务（不自启）：chess-robot-xiangqi-preview-20261031.service
停止实验：systemctl --user stop chess-robot-xiangqi-preview-20261031.service
再次启动：systemctl --user start chess-robot-xiangqi-preview-20261031.service
机械臂/夹爪服务不属于该实验服务。
相机配置：src/chess_robot_vision/config/d405.yaml；安装副本：install_vision/chess_robot_vision/share/chess_robot_vision/config/d405.yaml。
恢复自动曝光：将两份相机YAML的exposure_mode改为auto，再重启 chess_robot_preview.service；只会重启相机/预览。启动日志与FrameInfo可核对实际设置。
部署备份：deploy-backup-0-vision_node、deploy-backup-1-d405.yaml、deploy-backup-2-d405.yaml；源码备份vision-node-before.cpp。
源码/文档为现场未提交变更，没有提交或推送Git。

验证
C++相机包独立构建成功；colcon汇总3项测试零失败。额外检查米制反投影、空深度/零深度拒绝、深度离群值筛选及重叠框去重；三帧同步识别和基座变换成功。检查网页连续图像、曝光元数据、实际尺寸与帧号推进。

Notion来源
https://app.notion.com/p/3ff9f9a4e09e81fca280e1f3e4c42140
https://app.notion.com/p/4099f9a4e09e81339474e3b9a2a0066a
官方D405曝光说明
https://support.realsenseai.com/hc/en-us/community/posts/23848900451987-Setting-D405-Camera-Exposure-in-Python

计时口径审校：图像header的stamp为主机从SDK取出帧时记录的时间，早于对齐与校正，不是曝光时间。历史JSON即使使用capture_to_publish字段名，其发布延迟也是当前时刻减该header时间，未包括曝光至主机取帧之前的延迟；不能称为完整曝光到发布或曝光到显示延迟。同stamp验证消息交付一致性，不能替代硬件曝光同步精度验证。
