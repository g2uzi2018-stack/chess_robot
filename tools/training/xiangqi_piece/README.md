# 单类棋子预检测模型复现资料

本目录版本化2026-11-02实际执行的转换、训练、导出、评估及校验脚本；不自动启动训练或相机。
这些脚本记录固定Jetson工程与外部数据路径，不是通用安装器。原数据与模型不覆盖。

外部运行根目录：`/home/kuang/workspace/xiangqi-yolo-training/20261102-yolo26n-piece-v1`。
`python-piece.sh`记录该次已验证的局部依赖环境；原运行目录的`runtime/python-piece`使用同内容。
不要在生产监督器下直接执行这些训练脚本。GPU工作须沿用脚本中的阻塞互斥锁，退出释放；不更改生产时钟租约。

执行顺序为：`prepare_piece_dataset.py`转换239图/983实例并保留190/49分组；`train_piece.py`训练；
`export_piece.py`导出640/512/416严格FP32引擎；`evaluate_piece.py`和`compare_piece_backends.py`检查质量、速度与一致性；
`evaluate_piece_nms.py`及相关finish/review/verify脚本补查IoU>0.65去重与交付证据。
`finish_piece_pipeline.py`只编排已冻结运行目录，不能当成任意目录重训入口。

推荐640、conf=0.10、去重IoU=0.65。具体模型SHA与限制见
[验收说明](../../../docs/validation/xiangqi-precheck-20261102/README.md)和其中的`model-spec.json`。
模型二进制、数据和大型原始证据按现有仓库惯例保存在外部工件/本地运行目录，Git记录代码、参数和SHA。

检测模型无实例掩膜。正式接入需将候选框与原ROI分割匹配，并保留独立移动/覆盖检查；不能只替换`XIANGQI_MODEL_ARTIFACT`。
训练首版已完成不等于小目标、满棋盘、堆叠、分类或抓取验收通过。
