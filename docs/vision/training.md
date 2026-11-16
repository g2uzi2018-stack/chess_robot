# 棋子预检测模型训练

`tools/training/xiangqi_piece/` 保存单类候选模型的转换、训练、导出和评估工具。
当前脚本绑定 `/home/kuang/workspace/xiangqi-yolo-training/20261102-yolo26n-piece-v1` 的环境与数据。
这是一套有明确输入的复现入口；`finish_piece_pipeline.py` 编排该目录，不能作为任意数据的通用训练器。

1. `prepare_piece_dataset.py` 转换数据，`train_piece.py` 训练。
2. `export_piece.py` 导出，`evaluate_piece.py` 与 `compare_piece_backends.py` 比较结果。
3. `evaluate_piece_nms.py`、finish/review/verify 工具复核去重及工件资料。

依赖运行器为 `python-piece.sh`；训练数据和大模型位于上述外部目录。
现有配置为 640 像素输入、置信度 0.10、去重 IoU 0.65。候选检测器无实例掩膜，
须与 ROI 分割匹配使用，不能直接替换分割模型。沿用工具现有 GPU 锁，不修改现场时钟或服务。
