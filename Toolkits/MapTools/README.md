# 当前地图与道路信息工具包

2026-10-22 整理：下文历史表中的 legacy 现位于 `../archive/map_tools/legacy/`；旧调查 JSON 同样移至 `../archive/map_tools/`。本目录仅保留当前可用入口和说明。

从 newProject 根目录运行；只读取用户已加载的地图。默认不切图、不 tick、不生成车辆、不改灯。

```powershell
python -B -m map_tools --help
python -B -m map_tools --output map_exports/map_overview.json
python -B -m map_tools --junction-id 1427 --output map_exports/junction_1427.json
python -B -m map_tools --junction-id 1427 --draw --focus
```

先从 overview 的 junctions 选择 ID，再查看道路 ID、路口连接条数、信号 ID；单路口输出额外包含
movements 的 entry/exit 车道坐标、宽度、道路纵向 s，以及与采集器一致的 inventory。
顶层保存地图名称与 OpenDRIVE 指纹。导出文件必须不存在；地图重载后需重新导出。
工具导出是地图调查数据，不是 detection_config；不能把整份 JSON 直接当作检测线配置传入采集器。
选定后用 RoadSpec(junction_id=..., expected_map=...) 接入，或传给控制器 --junction-id。

--draw 仅绘制有限时间的路口及入口/出口标签，--focus 明确移动 spectator。
同步世界没有其他 tick 所有者时画面可能不会刷新，本工具不为刷新界面抢占 tick。
入口/出口标签是 junction movement 端点，不是真实停止标线，也不是全部道路曲线可视化。

## 旧文件用途与处理

| 原脚本 | 原用途 | 处理 |
| --- | --- | --- |
| showID.py、showNet.py | 地图路口标注/导出，但会切 Town05 或 tick | 当前工具包替代常用发现功能，旧源码移至 legacy |
| analyze_town05_junctions.py | 分析旧地图 JSON | legacy 保存，新的概览直接提供路口/道路摘要 |
| inspect_carla_junction_signals.py | 详细灯组调查 | legacy 保存；常用信号和停止参考信息纳入新导出 |
| draw_junction_road_connections.py、visualize_junction_stoplines.py | 旧可视化与观察相机，包含 tick | legacy 保存，新入口提供标签/视角；完整曲线路径绘制未迁入 |
| test_junction_signal_with_vehicle.py | 会改灯、冻结世界并生成车的早期试验 | legacy 保存，不属于地图只读工具，不纳入交付代码 |

旧 analyze 文本与 CSV 也归档至 legacy。原 junction_signal_inventory.json、town05_junctions.json
随目录更名保留在本地 map_tools 下，不作为新工具默认输入，也不进入精简交付包。所有 Markdown 保留。
本目录原名 codex，现统一更名为 map_tools；历史文档中的旧路径请按此对应。
