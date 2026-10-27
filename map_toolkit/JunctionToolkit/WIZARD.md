# 标准地图生成向导

在本目录运行 `generate_map.cmd`，或 `python wizard.py`。十字/规则网格固定三进三出与单车道单转向，仅选择名称、行列、路口间距、外围臂长和路肩。

非交互示例：`python wizard.py --type grid --rows 2 --columns 3 --name grid_2x3 --shoulder`。

默认输出到工具包父目录的 maps/，配置保存到 configs/generated/。生成完成后可显式选择加载 CARLA。新地图不会覆盖旧地图，不会复制到其他工程。

环境与详细规则见 [README.md](README.md) 和 [ROAD_STANDARD.md](ROAD_STANDARD.md)。
