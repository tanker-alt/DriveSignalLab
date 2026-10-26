# InformationCollector

负责当前帧状态采集、观测接口及采集运行配置。指标实现位于 `../EvaluationMetrics`，地图数据位于 `../Maps/MapDataset/maps`。

- `evaluation/`：状态采集、拓扑解析、世界快照及配置。
- `map_profiles/generated_cross3/`：当前Cross3运行必需的地图配置。
- 顶层控制/交通脚本：现有实验运行入口，保留以保障兼容。
- `../Toolkits/CollectionTools/`：停止线校准、地图配置、采集验证、交付工具。
- `../Toolkits/JunctionToolkit/`：地图生成、装饰及验证。
- `../Toolkits/MapTools/`：地图与路口检查。

历史输出、旧校准与归档已移至 `../LocalArtifacts/InformationCollector`，不上传GitHub；缓存和旧交付包已删除。过程文档位于 `../ProcessRecords`。
