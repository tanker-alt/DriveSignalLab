# 地图与通用路线适配器

本分支发布主工程 `maps/MapDataset` 当前 **10 张地图**和对应路线，源码、地图、manifest 与路线都在仓库内；不依赖原作者的绝对路径。
CARLA 服务端与 Python 客户端版本为 **0.9.15**，预处理使用 Python 3.7。

## 快速使用

```bash
git clone --branch maps https://github.com/tanker-alt/DriveSignalLab.git
cd DriveSignalLab
python -m pip install -r maps/route_adapter/requirements.txt
python maps/route_adapter/adapter.py verify
python -m unittest discover -s maps/route_adapter -p test_adapter.py -v
```

已经预处理，无须每次启动都重新构建。每张图含 `map.xodr`、`manifest.json`、配置/拓扑报告、`generated_routes.json`；环岛另含 `phase_example.json`。批次清单为 `MapDataset/route_release.json`。

|地图|类型|路线覆盖的物理路口数|
|---|---|---:|
|cross3|单四叉|1|
|t_north|单三叉|1|
|roundabout4_8groups|单环岛，完整一圈再驶出|4|
|cross_1x6 / cross_3x3|四叉路网|6 / 9|
|t_1x6 / t_3x3|三叉路网|6 / 9|
|heterogeneous_1x6 / heterogeneous_3x3|异构车道数路网|6 / 9|
|heterogeneous_3x3_roundabout|异构路网含环岛|12|

XODR 与 manifest 采用主工程已通过摄像头验收的远端灯头版本；道路和信号控制逻辑保持不变。
拓扑报告是随地图提供的既有记录；本次新生成路线的验证结果以 `route_release.json` 为准。

## 路线预处理

```bash
# 离线构建：只调用 carla.Map，不连接服务器，不改变正在运行的世界。
python maps/route_adapter/adapter.py batch
# 单张重建；发布整批时仍需 batch 更新批次清单和校验值。
python maps/route_adapter/adapter.py build --map-dir maps/MapDataset/heterogeneous_3x3
```

普通三叉/四叉/混合图使用连续车道规划器，异构图使用桥接及变道规划器，环岛使用整圈规划器。
输出按 CARLA 世界坐标、米为单位，记录地图 SHA256、规划源码 SHA256、起终点 road/lane/s、转向、变道与路口顺序。
每张图生成一条覆盖全部路口的连续路线；不是每条车道的路线库，也不是任意起终点寻路服务。

10/10 地图通过离线解析、地图哈希绑定、路口覆盖和采样连续性检查，最大相邻采样距离约 1.186 米（阈值 1.25 米）。
本次没有重新运行实车/多车仿真，因此路线明确标记 `physical_vehicle_validated: false`。几何连通不等于交通安全或控制策略验收。

## CARLA 运行时接口

结束当前实验后加载地图：

```bash
python maps/route_adapter/adapter.py load --map-dir maps/MapDataset/heterogeneous_3x3 --host localhost --port 2000
```

此命令会切换世界；有同步实验或车辆、行人、传感器时拒绝加载。默认执行内置美化：单路口 complete、路网 simple；加载后原生信号保持红灯，随后由控制器接管。美化失败直接报错，不回退到裸地图。
`load` 的服务器操作本次未重新实机验证；离线测试覆盖匹配地图下的路线读取、CARLA Location 和起点 Transform 转换。

供车辆路线跟踪器使用：

```python
from maps.route_adapter import load_route, carla_locations, spawn_transform

folder = 'maps/MapDataset/heterogeneous_3x3'
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
2026-11-09 已同步远端灯头修复。10 张图的 signal_control_geometry_validation.json 绑定当前地图哈希，证明原生停车 waypoint 和受控车道未变。stop_compensation_* 为保留原哈希的历史记录。

旧地图分支 `toolkit/urban-cross3-v1-20261027` 已弃用；地图生成、美化源码继续在 `toolkit/map-toolkit-2026-11-02` 维护。
本分支包含选定地图版本、预处理路线、运行时适配，以及生成和美化工具。


## 2026-11-05 停止线修复发布

当前清单为 10 张图；`t3_north` 更名为 `t_north`，旧 `mixed_1x6`、`mixed_3x3` 已归档，不再随本分支发布。旧版本仍可从 Git 历史获取。
每图附停止补偿 patch、validation 和 publication 报告。旧 admission/project-preflight/实车报告已失效的，不作为当前版本证据发布；experiment_profiles 必须重新生成。
所有路线均基于新 XODR 重新生成并绑定新哈希。单路口控制器旧默认路径为 t3_north，运行时请显式传 `--map-dir maps/MapDataset/t_north`。

美化和生成源码现已随本分支发布于 `maps/JunctionToolkit/`，来源为 D:/CarlaMapTools 的当前 standardization/JunctionToolkit，包括本次停止位置补偿版 generate.py、模板与配置。无需另外检出地图工具分支。

## 默认美化启动与地图生成

```bash
# 默认美化，单路口 complete，路网 simple；使用当前 CARLA Python 环境。
python maps/route_adapter/adapter.py load --map-dir maps/MapDataset/t_north
python maps/route_adapter/adapter.py load --map-dir maps/MapDataset/heterogeneous_3x3
# 直接美化入口，默认还拍摄预览；--no-photos 只跳过拍照，不关闭美化。
python maps/JunctionToolkit/view.py maps/MapDataset/cross3 --no-photos
```

运行环境：Python 3.7、CARLA Python 包/服务端 0.9.15。完整美化使用 CARLA 自带资产，不需要安装 RoadRunner。
地图生成使用独立 Python 3.12 环境，安装 `maps/JunctionToolkit/requirements-generator-lock.txt`，然后执行：

```bash
python maps/JunctionToolkit/generate.py --config maps/JunctionToolkit/configs/cross3.json --out maps/MapDataset/new_cross3
```

生成与 CARLA 环境分开安装。可在 `maps/JunctionToolkit/settings.local.json` 指定 generator_python 和 carla_python，勿提交机器专属设置。
工具随附历史说明中的 `standardization/JunctionToolkit` 路径，在本分支对应 `maps/JunctionToolkit`；地图输出使用 `maps/MapDataset`。
本次新增入口已验证默认美化分派和失败传播；未接管用户当前 CARLA 世界进行新一轮实机外观验收。


## 2026-11-09 远端灯头、生成器与美化更新

全部内容位于小写 `maps/`：地图在 `maps/MapDataset/`，生成器和完整美化模块在 `maps/JunctionToolkit/`，路线适配器在 `maps/route_adapter/`。

新增 signal_placement.py，普通路口、随机路网、环岛和异构拼接生成完成后统一设置独立物理灯头位置；保留逻辑 signal.s/validity。美化脚本按同一物理偏移布置支架。T 字封闭侧按实际路缘布灯，环岛按局部下游切线布灯。已验收地图不会在本次发布时重新生成道路。

10 张地图共 536 个信号、1,608 项车辆 RGB 检查全部通过；各地图 overview.png 和 street.png 均在验收后重拍。详见各地图 vehicle_camera_validation.json、far_signal_acceptance.json 和 reports/acceptance_summary.json。vehicle_camera/ 保留每个信号停车位置的全幅前视图。

相机：Lincoln MKZ 2020，1920×1080、水平 FOV 100°、局部 x=-1.5 m/z=2 m、俯仰 0°。检查停车位置和上游 15/30 m；短环内路段按实际可用距离采样并记录，不宣称完整 30 m 覆盖。标准为完整灯头入画和红/黄/绿真实渲染变化，不等于 ML 识别率或雨雾/夜间/交通遮挡认证。旧 900×256 配置的远距离灯色分辨率限制仍存在。

生成器回归重新生成了全部 10 类配置，验证物理姿态字段与 XML 一致、重复调用不累加偏移、既有验收地图不发生变化。新生成任意配置仍需独立 CARLA 验收。预处理路线已针对本次 XODR 重新构建和绑定哈希。
