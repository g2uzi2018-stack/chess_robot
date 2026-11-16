# 扫描与抓放

从项目根目录执行，先 `source tools/env.sh`。节点接口见 [ROS 接口](../ros_interfaces.md)。

## 1 配置和入口

| 配置 | 用途 |
| --- | --- |
| `config/scan/scan.current.json` | 20 cm 相机离台面、17 点扫描方案 |
| `config/scan/scan.site.json` | 现场演示采用的扫描配置；guard_file 保持与现有驱动参数一致 |
| `config/scan/scan.home-height.json` | 六点观察方案，非默认 |
| `config/pick/demo.current.json` | 叠放基线 |
| `config/pick/demo.row.current.json` | 排一行基线 |
| `config/pick/demo.row.live.json` | 当前工作区现场演示参数；不等同于两个基线 |

扫描仅规划：

```bash
ros2 run chess_robot_tasks scan_controller --config config/scan/scan.current.json --output runs/scan_NEW
```

执行扫描需要显式 `--execute`。正常完成回 Home 并确认停稳；故障或停止未确认时不自动发 Home。
每点按配置等待新鲜 scene 数据，确认条件由 `scan_controller.py` 和所选 JSON 决定；对应规则在所选配置和源码中维护。

抓放和排一行使用 `bash src/chess_robot_tasks/scripts/run_once.sh --config <配置> ...`。
其中 `--execute` 模式要求当前授权说明及新鲜扫描，不接受历史扫描替代本轮测量。
日常演示、自动准备、正常停机及恢复命令见 [演示操作](demo-speed.md)。

## 2 数据与状态

业务输出包括所选配置、动作事件、逐视点测量和结果，保存到指定 `runs/<批次>/`。
扫描清单是搜索提示，不能代替抓取前的新鲜测量。任务配置绑定当前安装、Home、标定和几何文件身份。
手眼采用值保存在 `data/handeye/`；修改路径时保持原文件内容及绑定哈希一致。

`run_speed_demo.sh` 仍调用现有现场机械臂服务及其运行配置；工作区源码和已运行节点分别管理。
工作区与现场冻结视觉版本存在差异，不能把本次开发构建视为整机功能验收。
