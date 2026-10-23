# AG95 夹爪模型

从 `g2uzi2018-stack/nova` 的 AG95 description 包迁入，保留 Apache-2.0 许可证。
这是模型资源包，提供网格、关节和抓取坐标系；不包含串口驱动或控制器。

- `meshes/visual/`、`meshes/collision/`：各 6 个原始 STL，字节不变。
- `urdf/dh_ag95_macro.xacro`：原始宏，字节不变；包含 mimic 手指联动、惯性和历史碰撞补丁。
- `urdf/ag95.urdf.xacro`：新增的独立预览入口，无硬件依赖。
- `urdf/ag95.urdf`：由入口生成的纯 URDF，浏览器无需展开 Xacro。
- `SOURCES.json`：来源路径、提交和 SHA-256；`LICENSE` 为原始 Apache-2.0 许可证。

原宏的 STL 缩放为 `0.001`，坐标长度使用米。关节角范围、惯性和碰撞补丁均保留旧值，
迁移不等于实机行程、负载或碰撞精度验收。不要把驱动的开度命令直接当作弧度。
组合模型沿用旧项目的 `grasp_link_offset=0.20`，需要用当前实物核对。

浏览器查看和模型生成步骤见 `../../docs/models/ag95-browser.md`。
