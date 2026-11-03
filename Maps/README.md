# 地图与通用路线适配器

本分支发布主工程 `Maps/MapDataset` 当前 **12 张地图**和对应路线，源码、地图、manifest 与路线都在仓库内；不依赖原作者的绝对路径。
CARLA 服务端与 Python 客户端版本为 **0.9.15**，预处理使用 Python 3.7。

## 快速使用

```bash
git clone --branch maps https://github.com/tanker-alt/DriveSignalLab.git
cd DriveSignalLab
python -m pip install -r Maps/route_adapter/requirements.txt
python Maps/route_adapter/adapter.py verify
python -m unittest discover -s Maps/route_adapter -p test_adapter.py -v
```

已经预处理，无须每次启动都重新构建。每张图含 `map.xodr`、`manifest.json`、配置/拓扑报告、`generated_routes.json`；环岛另含 `phase_example.json`。批次清单为 `MapDataset/route_release.json`。

|地图|类型|路线覆盖的物理路口数|
|---|---|---:|
|cross3|单四叉|1|
|t3_north|单三叉|1|
|roundabout4_8groups|单环岛，完整一圈再驶出|4|
|cross_1x6 / cross_3x3|四叉路网|6 / 9|
|t_1x6 / t_3x3|三叉路网|6 / 9|
|mixed_1x6 / mixed_3x3|混合路网|6 / 9|
|heterogeneous_1x6 / heterogeneous_3x3|异构车道数路网|6 / 9|
|heterogeneous_3x3_roundabout|异构路网含环岛|12|

原始 XODR 与 manifest 按字节复制自主工程，没有为了生成路线修改几何、信号或停止位置。
拓扑报告是随地图提供的既有记录；本次新生成路线的验证结果以 `route_release.json` 为准。

## 路线预处理

```bash
# 离线构建：只调用 carla.Map，不连接服务器，不改变正在运行的世界。
python Maps/route_adapter/adapter.py batch
# 单张重建；发布整批时仍需 batch 更新批次清单和校验值。
python Maps/route_adapter/adapter.py build --map-dir Maps/MapDataset/mixed_3x3
```

普通三叉/四叉/混合图使用连续车道规划器，异构图使用桥接及变道规划器，环岛使用整圈规划器。
输出按 CARLA 世界坐标、米为单位，记录地图 SHA256、规划源码 SHA256、起终点 road/lane/s、转向、变道与路口顺序。
每张图生成一条覆盖全部路口的连续路线；不是每条车道的路线库，也不是任意起终点寻路服务。

12/12 地图通过离线解析、地图哈希绑定、路口覆盖和采样连续性检查，最大相邻采样距离约 1.186 米（阈值 1.25 米）。
本次没有重新运行实车/多车仿真，因此路线明确标记 `physical_vehicle_validated: false`。几何连通不等于交通安全或控制策略验收。

## CARLA 运行时接口

结束当前实验后加载地图：

```bash
python Maps/route_adapter/adapter.py load --map-dir Maps/MapDataset/mixed_3x3 --host localhost --port 2000
```

此命令会切换世界；有同步实验或车辆、行人、传感器时拒绝加载。只加载原生道路，不添加美化标线；原生信号暂保持红灯，随后由控制器接管。
`load` 的服务器操作本次未重新实机验证；离线测试覆盖匹配地图下的路线读取、CARLA Location 和起点 Transform 转换。

供车辆路线跟踪器使用：

```python
from Maps.route_adapter import load_route, carla_locations, spawn_transform

folder = 'Maps/MapDataset/mixed_3x3'
route = load_route(folder, world)          # 核对地图和当前世界
points = carla_locations(folder, world)  # list[carla.Location]
start = spawn_transform(folder, world)   # 起点离地 0.3 m
# 将 points 交给项目自己的轨迹跟踪器；生成车辆前自行检查起点占用。
```

适配器不生成车辆、不推进世界、不设置 Traffic Manager 种子、不执行信号策略。不要把坐标点直接当成 Traffic Manager 的转向字符串或 LocalPlanner 的 `(Waypoint, RoadOption)` 序列；轨迹执行和信号遵守由接入方负责。

## 配合控制器分支

```bash
git fetch origin feat/single-junction-signal-controller
git restore --source=origin/feat/single-junction-signal-controller -- signal_control
```

单路口控制契约见 `signal_control/README.md`。路网统一控制仍需同事实现；地图路线适配器不替代路网红绿灯适配器。
当前停止线美化位置与控制/评估停车参考尚未统一标定，不能据此宣称正式红灯越线指标已验收。

旧地图分支 `toolkit/urban-cross3-v1-20261027` 已弃用；地图生成、美化源码继续在 `toolkit/map-toolkit-2026-11-02` 维护。
本分支只负责选定地图版本、预处理路线及运行时读取适配。
