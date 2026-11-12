# 2026-11-03 小模型全图候选上线

YOLO26n单类detect已接入生产，替代原分割模型的全图候选pass；原YOLO26s-seg继续做同帧ROI的14类分类与实例轮廓。候选框保留在原图平面，只有唯一匹配且完整覆盖候选的同帧ROI轮廓才参与原深度表面定位。无匹配轮廓保留unknown/几何无效，不用矩形代替掩膜。

## 现场入口

- 预览：[相机预览](http://192.168.30.26:8088)，选择“检测叠加图”；此地址限能访问该局域网的客户端。2026-11-03实际GET首页与通道API均返回200。
- 视觉user service：`chess-robot-xiangqi-preview-20261031.service`。
- 项目入口：`runs/project_release_20261102_01/deployment/start_visual.sh`。
- 当前冻结源码和记录：`runs/xiangqi_lite_deploy_20261103_01/{source,production}`。
- 只切换视觉服务，保留已有时钟租约；相机、绘制进程与机械臂配置/命令审计保持，无机械臂或夹爪动作。

## 最终版本60秒实测

窗口2026-11-03 09:46:24（Asia/Taipei）起60.000秒，1418个独立图像时间戳、无重复帧，每帧4个候选/4个输出/1个ROI。

| 指标 | 结果 |
| --- | --- |
| 实际处理吞吐 | 23.63fps |
| 网页recognition中位数 | 23.58Hz |
| 网页VIDEO中位数 | 30.00Hz |
| 相机FrameInfo到达率 | 29.98Hz |
| 四枚类别符合当前截图 | 1407/1418帧，99.22% |
| 四枚深度几何同时有效 | 1332/1418帧，93.94% |
| 主机SDK取帧stamp到发布延迟p50/p95 | 120.93/139.63ms |
| GPU阶段p50/p95 | 35.55/36.91ms |
| 其中小模型候选p50/p95 | 7.80/8.61ms |
| 几何worker p50/p95 | 13.52/15.25ms，与下一帧GPU重叠 |

其余11帧黑象没有匹配到ROI轮廓/类别，输出unknown并撤销其几何资格。吞吐、类别一致率、几何通过率和快照可领取频率分别统计；23.63fps不表示每帧四子全部正确。

此前120秒窗口为23.65fps；最后一次多ROI掩膜覆盖检查修正后的本表60秒确认没有性能回退。旧方案此前17.78fps时是直线四子，当前已改为聚拢四子，不能当作严格同布局A/B，也不能将75.4%与99.22%直接比较宣称分类精度提高。当前类别参考是本次截图中人工核对的红兵、黑卒、黑象、红兵（按x排序），不是独立数据集泛化准确率。

## 模型和配置

`XIANGQI_CANDIDATE_MODE=lite`、`XIANGQI_CANDIDATE_ARTIFACT`、`XIANGQI_CANDIDATE_ARTIFACT_SHA256`显式指定候选工件，`XIANGQI_CANDIDATE_CONF=0.1`。原`XIANGQI_MODEL_ARTIFACT`仍指向14类分割引擎，不能换成此detect-only模型。

候选输入原图1280×720，静态TRT profile `[1,3,384,640]`、rect=True、FP32/TF32关闭、按置信度降序IoU>0.65去重。该engine不能用于任意正方形ROI。模型外置，不进Git；路径与SHA见[模型配置](model-spec.json)。

policy `lite_candidates_roi_classes/v1`沿用`xiangqi_camera_measurements/v2`消息，新增候选引擎身份/调用计数及实例轮廓来源；perception与scene须配套使用新源码。旧`yolo`模式保留兼容，源码默认不等于现场显式配置。深度预检测仍仅为shadow，不是现行生产方案。

## 检查和交付边界

最终Jetson现有83项回归通过（5.78秒）；12组历史RGB-D的候选关联、ROI轮廓、深度和消息契约均通过。补充临时6类关联场景、4种伪造来源拒绝和2种多ROI覆盖顺序检查通过，临时验证脚本用后删除，未新增持久化测试文件。此提交只增加实现和验收记录，没有增加测试脚本。

[紧凑结果及独立回读](results.json)记录最终60秒指标、模型身份、源码SHA和部署审计；原始逐帧记录在运行目录`live-final-records.json.gz`，本地交付包`xiangqi-lite-deployment-20261103.zip`。运行目录和工件不随Git分发。

满棋盘/拥挤遮挡、真实绝对定位、法向/yaw、棋盘检测和抓取闭环尚未验收。单类候选模型训练标签只有piece，不能宣称已实现棋盘检测。原定位/快照门限保留，`motion_ready`和`grasp_ready`仍false。

## 显式回退（本次未执行）

```bash
python3 /home/kuang/workspace/chess_robot/runs/xiangqi_lite_deploy_20261103_01/rollback_lite.py
```

脚本要求当前启动器仍是本版，恢复备份的项目视觉入口并仅重启视觉service；原发布源码/运行目录保留。该命令不会控制相机、网页或机械臂服务。本记录不是开机自启、动作使能或抓取验收。
