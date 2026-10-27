# 独立 CARLA 地图工具包分支

从 [JunctionToolkit/README.md](JunctionToolkit/README.md) 开始。

- [统一道路规范](JunctionToolkit/ROAD_STANDARD.md)
- [实机验收记录与边界](JunctionToolkit/STANDARD_VALIDATION.md)
- [统一美化模板与批量生产](BEAUTIFICATION_PIPELINE.md)
- [十字路口预览](examples/cross3/overview.png)
- [带路肩网格预览](examples/grid2x2/overview.png)

本目录以独立工具包方式增加到仓库分支，未替换项目原有模块。生成的地图默认保存在本工具包内 maps/。源工程保留在本机 CarlaMapTools 工作区。

examples 是经检查的轻量 XODR/配置/报告/RGB 示例，不是已烘焙 CARLA 地图包。models、贴图、CARLA/RoadRunner 安装内容与虚拟环境不在此包内。没有新增外部资产授权；沿用仓库许可并尊重第三方组件各自许可。
