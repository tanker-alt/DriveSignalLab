# 美化模式

统一入口：`view.py MAP_DIR [--beautify MODE]`，也可通过 `toolkit.py view` 调用。

|模式|作用|
|---|---|
|`auto`（默认）|多路口网络采用简化模式；单个四叉、三叉和环岛采用完整模式|
|`simple`|只添加停止线与信号杆，保留原生可控信号灯|
|`complete`|采用原有完整美化流程|
|`off`|不添加运行时美化|


多路口简化模式的独立入口为 `network_simple_finish.py MAP_DIR`；支持 `--host`、`--port` 和 `--no-photos`。需要完整路网美化时使用 `view.py MAP_DIR --beautify complete`。

在主工程根目录运行，例如：

```powershell
& D:/CarlaMapTools/carla-env/Scripts/python.exe -B Toolkits/JunctionToolkit/view.py maps/MapDataset/mixed_1x6
& D:/CarlaMapTools/carla-env/Scripts/python.exe -B Toolkits/JunctionToolkit/view.py maps/MapDataset/mixed_1x6 --beautify complete
```

简化模式使用 manifest 中的停止线参考位置，绘制无碰撞的白色停止线，保留原生信号头并使用现有标准信号杆。不添加运行时斑马线、整路标线、草地、树木或家具。地图本身的道路、人行道、原生标线及拓扑仍由 OpenDRIVE 加载。单个环岛虽包含多个 CARLA junction 控制节点，仍按单个路口默认完整美化。

`load_report.json` 的 `appearance_mode` 记录实际采用的模式。后续新生成的地图使用同样的加载器默认规则，无需修改地图几何或重新生成道路。
