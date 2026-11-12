# 过时文档入口迁移记录

日期：2026-11-02。此页记录文档整合的路径和原文身份；历史验证结果本身不作追溯改写。

## 1 已由单一入口替代的六个路径

| 已移除路径 | 替代入口 |
|---|---|
| `docs/models/urdf-handoff.md` | [docs/models/urdf-workflow.md](../models/urdf-workflow.md) |
| `docs/validation/business-node-2026-10-25.md` | [docs/validation/business-node-20261025/REPORT.md](business-node-20261025/REPORT.md) |
| `docs/validation/business-node-20261025/QUICKSTART.md` | [docs/operations/arm-business-acceptance.md](../operations/arm-business-acceptance.md) |
| `docs/validation/power-handoff-tolerance-20261029.md` | [docs/validation/reference-recovery-history.md](reference-recovery-history.md) |
| `docs/validation/power-handoff-recorded-20261029.md` | [docs/validation/reference-recovery-history.md](reference-recovery-history.md) |
| `docs/validation/power-handoff-recorded-dwell-fix-20261029.md` | [docs/validation/reference-recovery-history.md](reference-recovery-history.md) |

前三类为URDF交接/工作流及业务节点跳转的重复入口；三份power-handoff实现报告合并为参考恢复历史。替代页保留恢复契约、数值、失败修复边界和原始证据路径。

## 2 原文、校验和恢复

原文按原相对路径保存在 `runs/docs_cleanup_20261031_01/originals/`；同目录 `manifest.json`、`baseline-documents.json` 与 `RESTORE.md`记录备份和恢复方法。该目录被Git忽略，发布/备份时需单独保留。Git历史还保留已提交旧版本；未提交旧字节以原文备份为准。

| 原路径 | 原文SHA256 |
|---|---|
| `docs/models/urdf-handoff.md` | `f2b39a2803cc118433a726ff3e0c5cc46ce635f55889a9a308544b0889b5430b` |
| `docs/validation/business-node-2026-10-25.md` | `9e1d1010956f114c1b900e651bbdb4f1be5bf6f6a10cf27cd1390cb9df82ec53` |
| `docs/validation/business-node-20261025/QUICKSTART.md` | `a33a72800730d145d9129d99ed4091719d5bdb9d5e0f1c6c0cea2965b7c3f328` |
| `docs/validation/power-handoff-tolerance-20261029.md` | `d59469cf76b831f1b033259fecad6a83d1ec46688acbbdc037edde8b40417bc4` |
| `docs/validation/power-handoff-recorded-20261029.md` | `a4a1d546b748dd5f196d2c5320ca91df69b7d07ee8b2db7703d946d40e1be571` |
| `docs/validation/power-handoff-recorded-dwell-fix-20261029.md` | `e5e1275095a9f13aea7aa3e144db70ed62494362794e37b4351d16a1787d2fa6` |

需要追溯时先校验原文SHA，再阅读冻结上下文。若恢复文件，先比较当前修改并逐文件恢复，不对整个仓库执行reset/clean。旧候选部署命令不是现用启动流程。

## 3 保留范围

- 厂家资料、许可证、原始标定/RGB-D、完整审计和唯一验收/失败记录保留。
- 其余按日期冻结的验证报告继续保留，由验证索引说明适用范围；旧日期本身不是删除理由。
- 旧manual_panel服务的可复制启动段已从活跃panel说明中移除，历史来源和验证范围仍可追溯。
- 更早清理记录中的8项删除包含两份原本未追踪的handeye-laser-recovery-summary HTML/TXT；它们归并入手眼结果页，也有原文备份，因此与本轮Git所见6个删除不矛盾。
