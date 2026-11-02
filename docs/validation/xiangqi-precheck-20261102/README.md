# 2026-11-02 预检测、ROI与轻量模型验证

同帧全图候选＋ROI类别策略已部署；未知、冲突与未覆盖候选不获场景资格，原动作/抓取资格仍为false。
生产运行使用冻结源码，Git提交或push不会自动切换该版本。

深度模式仅shadow。真实12帧输出4–11候选，算法覆盖0/12；Orin深度CPU约76ms，深度＋ROI约216ms，未上线。
二维边缘＋椭圆仅在同12帧静止无棋盘场景试验，Orin约2.6ms，不作为完整场景验收。

轻量模型首版：YOLO26n detect、单类piece、640长边/实际1×3×384×640、严格FP32禁TF32、conf=0.10、IoU>0.65去重。
原239图/983实例自动转换，保持190/49分组，训练50轮约9.7分钟；三尺寸PT/TRT检查通过。
640内部验证198/198检出、1个标注外框、0重复；已知四子回放12/12帧恰4候选。
640独立组合：候选7.94/8.17ms，候选＋原ROI35.02/36.06ms；旧方案48.17/53.32ms（p50/p95）。
此为离线阶段耗时，不是完整ROS帧率。原ROI分类歧义未修复；新模型尚未生产接入。

详见`lite-report.txt`、`lite-results.json`、`model-spec.json`、`depth-report.txt`和`depth-review.json`。
完整模型/证据包已交付用户，二进制保留在外部工件目录，路径和SHA在model-spec中。

## 多目标与关灯测试

固定网络输入尺寸下，目标数对卷积前向计算量影响较小；候选后处理、输出掩膜、几何与关联成本会随目标数增加。
目标分散或棋盘场景需要多个分类ROI时，会增加原分割模型的推理次数；四子单ROI不能外推满棋盘吞吐。
应以同样的质量规则检查1/4/8/16/32子与不同分布，记录候选数、ROI数、各阶段p50/p95及完整独立帧吞吐。

关灯不影响用已保存的亮灯RGB-D/内参进行离线推理与几何回放，但回放不验证当前相机曝光、采集和实时发布。
暗场漏检会减少后续工作量，暗场高帧率不能证明多目标性能。
完整实时验收需照明正常且真实目标可见；固定手动曝光下关灯不必然改变相机取帧频率。

## 检查

CPU回归：`python -m pytest tools/testing/xiangqi_tracking/test_candidate_roi.py tools/testing/xiangqi_tracking/test_depth_candidates.py tools/testing/xiangqi_tracking/test_depth_policy.py tools/testing/xiangqi_tracking/test_node_split.py -q`，合计83项。
继承回归52项与此部分重叠，Windows有Linux/ROS限制；不能相加当独立测试数。
`benchmark_lite_roi.py`只在独立GPU锁下做保存图像的阶段对比，不发布ROS、不授予几何资格。
