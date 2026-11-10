# Route adapter：连续路线与多 OD 拓扑接口

本目录基于 DriveSignalLab 已发布的 `map_toolkit/standardization/route_adapter` 扩展。既有 `load_route`、`carla_locations`、`spawn_transform` 接口及 `build/batch/verify/load` 命令保留；新增功能不覆盖 `generated_routes.json`，不改变原连续路线格式。

本地路径为 `Maps/route_adapter`；发布仓库使用 `map_toolkit/standardization/route_adapter`，位于 **maps 分支**。本工具面向 CARLA/OpenDRIVE 的拓扑与 OD 路线适配，不是通用导航库。以下示例以发布路径为准。依赖 CARLA Python API 0.9.15；离线接口只构造 `carla.Map`，不创建 `carla.Client`。

## 安装与旧接口

验证环境：Windows、Python 3.7、CARLA Python API 0.9.15。其他 Python 版本必须有匹配的 CARLA 0.9.15 wheel，尚未在本次发布验证；不要把生成器的 Python 3.12 环境用于 CARLA 调用。仅加载已有连续路线 JSON 不需要导入 CARLA；OD、几何转换、重建需要该 Python API，仍不需要 Server。动态 `load` 需要同版 CARLA Server 和仓库内 JunctionToolkit 依赖。

```shell
git clone --branch maps https://github.com/tanker-alt/DriveSignalLab.git
cd DriveSignalLab
python -m pip install -r map_toolkit/standardization/route_adapter/requirements.txt
python map_toolkit/standardization/route_adapter/adapter.py verify
python -m pip install -r map_toolkit/standardization/route_adapter/requirements-test.txt
python -m unittest maps.route_adapter.test_adapter maps.route_adapter.test_topology_api maps.route_adapter.test_contract
```

```python
from map_toolkit.standardization.route_adapter import load_route, carla_locations, spawn_transform
folder = 'maps/MapDataset/cross3'
existing = load_route(folder)  # 读取原 continuous_route.4，不重新规划
locations = carla_locations(folder)  # list[carla.Location]
# transform = spawn_transform(folder, world)  # 运行时核对地图再取得出生点
```

旧 `build/batch/verify/load` 参数继续可用；`build/batch` 会重建原连续路线文件，`verify` 只读，`load` 会连接服务器并加载世界。

## 新增公开接口

```python
from map_toolkit.standardization.route_adapter import adapt, build_od_catalog, audit_route

folder = 'maps/MapDataset/cross3'
result = build_od_catalog(folder)
row = result['route_catalog'][0]
check = audit_route(folder, row['route'],
                    (row['destination'], row['destination_lane']))
```

- `adapt(folder)`：输出 `map_adapter.2`，包括地图哈希、入口/出口、OD、路线及检查结果、信号、停止线。先验证 manifest 与 OpenDRIVE 哈希一致；信号/停止线映射错误保留在 `diagnostics`，`passed=false`。不可构建拓扑时抛出 `AdapterError`（`ValueError` 子类）。
- `build_od_catalog(folder)`：返回同一结构；目录为空、任一路线检查失败或映射有 error 时抛出异常。只存在 warning 不拒绝拓扑结果。
- `audit_route(folder, route, expected_exit=None)`：审计 CARLA 世界坐标与 road/lane/s 路点；返回 passed、errors 和几何标签差异警告。
- `lane_s_direction`、`transition_route`、`bypass_route`：提供方向解析和合法相邻车道候选路径构建；需要离线或运行时 CARLA Map，不控制车辆。

每个 OD 行包含 route_id、origin、lane、destination、destination_lane、route。路点包含 x/y/z/yaw、road_id、lane_id、section_id、s。坐标为 CARLA 世界米制坐标，不是 OpenDRIVE 原始 XY。

```shell
python map_toolkit/standardization/route_adapter/adapter.py od --map-dir maps/MapDataset/cross3 --output od_catalog.json
```

`od` 完全离线；省略 output 时仅打印摘要。只有既有 `load` 命令会连接服务器和加载地图。

## 输入与输出契约

输入目录必须包含 `map.xodr`、具有 `opendrive_sha256` 的生成器 `manifest.json`。可选 `safety_rules.json` 一旦存在，其哈希必须与地图一致。没有停止线时可使用 manifest 中的 `reference_s`，并通过 `position_source` 标注来源；这不是停车位置实车标定。无信号映射的地图会返回 warning、`signal_stop_mapping=false`，不能声称已适配信号控制。

机器可读契约见 [output.schema.json](output.schema.json)，使用 JSON Schema Draft 7。兼容版本仍为 `map_adapter.2`，消费者应允许新增字段；`passed`、`diagnostics` 和能力标记是本次补充。`classification.kind` 保留生成器类型（cross、t、roundabout、heterogeneous、random_grid 等）；`road_count` 包括连接器道路。

|字段|含义|
|---|---|
|opendrive_sha256|原始 map.xodr 字节 SHA256|
|entries / exits|边界入口/出口的 road_id、lane_id|
|route_catalog|每个可达边界 OD 一条确定性 successor 路线|
|od_connections|route_id 与入口/出口车道绑定，与目录逐项对应|
|topology_checks|逐路线 passed、errors（index/reason）及几何标签警告|
|signals|signal_id、junction_id、road_id、lane_id、reference_s|
|stop_lines|停止线 id、signal_opendrive_id、road_id、lane_id、s_m|
|diagnostics|code、detail、severity（error 或 warning）|
|capabilities|拓扑/映射能力；特殊路权和动态验证始终为 false|

最小消费示例：

```python
from map_toolkit.standardization.route_adapter import adapt, AdapterError
try:
    data = adapt('maps/MapDataset/cross3')
    print(data['schema_version'], data['passed'])  # map_adapter.2 True
    print(len(data['route_catalog']))             # 12
    for issue in data['diagnostics']:
        print(issue['severity'], issue['code'], issue['detail'])
except AdapterError as exc:
    print(exc.code, exc.detail)
```

稳定失败码包括 `unresolved_connector`、`unresolved_connector_lane`、`unresolved_road_link`、`unresolved_successor`、`lane_direction_unresolved`、`lane_end_unresolved`、`lane_sample_unresolved`、`no_boundary_od`、`unreachable_boundary_exit`、`boundary_road_too_short`。映射诊断包括 `signal_mapping_unresolved`、`signal_lane_unresolved`、`stop_signal_mapping_unresolved`、`stop_lane_unresolved`、`signal_stop_missing`、`stop_line_map_hash_mismatch`。特殊路权固定输出 `special_right_of_way_unsupported` 警告，不把普通顺向拓扑成功解释为特殊路权合法。

## 范围与限制

- 原连续路线用于遍历路口；新 OD 目录枚举边界入口到可达边界出口，每个目标保留一条连接路径，不枚举所有替代路径。
- OD 目录沿车道 successor 行驶，不主动换道；默认距离地图边界 15 米起终点。短道路或无法解析的连接应报错。
- 没有边界的闭合路网不能直接得到边界 OD，需后续提供显式起终点接口。
- 审计是中心线路径和车道拓扑检查，不证明车身无碰撞、控制器可执行或遵守信号。
- 特殊授权逆向借道不能按普通顺向车道检查直接通过；授权规则留在场景层。
- 大路网的 OD 枚举成本随入口、出口和路网大小增长，发布前须审查兼容性报告。
- BFS 每个入口对每个车道段最多入队一次，仍需为所有 OD 生成逐米采样；不是最短米制距离优化。每个方向只有一条路径，不主动换道，不枚举替代路径。
- XML 引用预检查不能替代完整 OpenDRIVE XSD 验证。对第三方文件建议使用下述进程隔离矩阵，原生 CARLA 解析器错误可能终止当前进程。

## 测试与接入

```shell
python -m unittest maps.route_adapter.test_adapter maps.route_adapter.test_topology_api maps.route_adapter.test_contract
python map_toolkit/standardization/route_adapter/compatibility.py --dataset maps/MapDataset --output compatibility.json --timeout 60
```

旧接口测试需要发布地图包中的 generated_routes.json 和 route_release.json。可通过环境变量 `ROUTE_ADAPTER_DATASET` 指向完整发布数据集。测试中的地图加载使用 mock，不会实际连接服务器。

本地场景工程的 tools/map_adapter.py 与 tools/route_topology.py 已改为兼容导入桥接；核心实现仅在本工具包维护。发布时只包含工具包源码、测试、依赖和说明，不包含本地专项语义判据或验收输出。

`compatibility.py` 对每张地图启动独立进程，区分适配失败、worker 异常退出、超时、基础设施失败；每图结束原子更新报告。报告包括源码指纹、逐图地图哈希、OD 数量、耗时和失败原因，完成前 `completed=false`。报告属于使用者本地证据，不随工具包提交。当前发布验证范围见 [CHANGELOG.md](CHANGELOG.md)。
