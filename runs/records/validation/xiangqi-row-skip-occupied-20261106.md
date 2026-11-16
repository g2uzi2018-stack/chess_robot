# 2026-11-06 占位保留、跳槽与重新启动

用户修改规则：摆放位有棋子时跳过该摆放位，并重新启动程序。现场配置 row_occupied_policy=skip，中文说明明确：已有行内棋子保留，不作为来源，不算本轮新摆。旧配置默认 occupant_first 兼容原策略。

## 实现

RowDemo 将 slot_index 槽位进度与 layers / placements 新放数量分开，skipped_slots 记录确认占位证据。粗扫仅为提示，若疑似占位，先到高观察位取得新鲜完整几何确认；确认后跳槽。没有粗扫提示时仍保留持物放前和最终下降发送前的空槽检查。

RealIO 使用 RowSlotOccupied 传递明确占位和对应 RGBD 帧/位置/身份。未知、不完整检测仍是视觉不确定，继续主动观察分支或拒绝，不能伪造跳槽。场景身份/覆盖不合格不会变成占位证据。

若持物放前或最后发送前发现占位，跳过该槽；若曾下探观察，先受检回搬运高度，再用原持物包络检查转移到下一位，只释放一次。若此时所有剩余槽位都被占，保留夹持故障退出。

所有目标行位置从来源候选中排除，避免“后面的占位棋子又被抓走”。来源仍按当前空槽最近优先。新鲜确认的占位从待处理来源提示中退役展示，但不冒充本轮放置。

## 验证

policy-validation.json 的 8 项内存编排验证通过：占位跳槽且独立计数；全部占位零抓取；粗提示误报不跳；持物后发现占位转移同一颗；发送前竞争占位不执行该下降；所有后续占位保留夹持；未知检测不跳；整行来源保护及旧策略兼容。

adapter-validation.json 的 2 项接口验证通过：新鲜完整几何生成结构化占位证据；身份变化不会冒充占位。全部测试以内联方式执行，未生成持久化测试源。

已重新启动原 row --workspace-ready 业务，沿用有效参考、已初始化空爪和 0.5/0.5 速度配置。最终运行结果见下节。证据目录 runs/row_skip_occupied_20261106_01。

## 实机结果

首次重新启动在 p14a 扫描时 ServoJ 应答超过原 150 ms 限值而停止；Stop 已确认，参考与干净检查点均保留。从独立确认的停稳状态受检回 Home，再启动全新业务，没有重发原失败动作或修改超时阈值。

第二次因远程 stdout 管道断开而退出，参考有效、空爪、mode=5、停稳且检查点干净。随后改用独立 user unit chess-robot-row-skip-20261106.service 运行，标准输出/错误写 business.log，Restart=no；终端连接断开不会切断任务日志。

第三次完整 17 点扫描结束后，新鲜确认并跳过前 4 个槽位，skipped_slots 的 slot_index=[0,1,2,3]，placed_count=0，next_slot_index=4。没有新的 GRASP，原行内棋子没有重抓。其余候选仍因 neighbor/unknown too close for reviewed finger envelope 而跳过，本轮没有新增摆放；不能把跳过的 4 槽算成本轮新摆 4 颗。

最终正常 PARTIAL，最后 HOME Action 成功；空爪 raw=1000、reference_valid=true、controller_mode=5，检查点 restorable=true。row_slots_processed=false：第 5–8 槽没有新来源可抓，尚未处理完。没有为了跑完而降低原净空要求。

结构化结果 result-summary.json，最终完整 run-result 路径记录其中。占位跳过已实机验证；持物后转移到下一空槽及发送前竞争占位已通过内存编排验证，本轮未实际触发。
