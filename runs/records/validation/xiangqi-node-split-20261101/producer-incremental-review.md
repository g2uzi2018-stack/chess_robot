# Producer-only narrow incremental review

结论：本次指定producer标量归一与部署辅助脚本增量无阻塞。

manifest-02.json 的22项哈希均与final-source-02匹配。相对上一版measurement_contract，唯一行为增量是新增producer_json_value，在camera_measurement内部生成packet后归一str子类和NumPy bool/integer/floating标量，再进入原strict encode。独立AST比较check_json、encode_json、decode_json、validate、decode完全不变，外部输入边界没有放宽。

独立纯CPU测试：模拟TorchVersion的str子类，以及np.bool_/int64/float32均归一为原生标量；NaN、Inf、-Inf、NumPy NaN、ndarray、任意object六项均ContractError拒绝；向外部check_json直接传str子类/NumPy标量仍被拒绝。未读模型或原始现场图像，也未运行推理。

阅读test_producer_scalars.py及producer-scalars.json/producer-scalars-trt.json：真实Experiment(camera_only=True,geometry_only=True)构造、cached detections形成record；唯一非原生路径为identity.torch/TorchVersion，归一后零非原生路径。TRT evidence明确inference_backend=tensorrt且inference_executed=false，不能把它当真实推理或实时验收。

deployment_helper本轮SOURCE改source-final-02，SOURCE_MANIFEST独立production-source-manifest-02.json；既有BACKUP只复用不覆盖，prepare要求TOP先等于原backup。独立读取TOP和原backup字节一致。clock lease等其余生命周期沿用上一轮已审结构；未执行prepare/apply/rollback或ROS/硬件命令。

哈希、测试与限制详见producer-incremental-review.json；此前报告保留。
