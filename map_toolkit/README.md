# 为 CARLA 0.9.15 生成、验证和美化可连通的 OpenDRIVE 路口与随机路网。

## 环境

当前验证环境为 Windows，命令示例使用 PowerShell，并在仓库根目录执行。

|组件|版本及用途|
|---|---|
|CARLA 服务端|0.9.15，独立安装并启动|
|生成环境|Python 3.12；scenariogeneration 0.16.7|
|CARLA 客户端环境|Python 3.7；carla 0.9.15，必须与服务端匹配|
|默认连接|localhost:2000|

两套 Python 环境分别处理道路生成和 CARLA API，不要合并安装。运行时美化使用 CARLA 自带资源；常规生成与加载不要求安装 RoadRunner，也不需要复制商业资源包。

有效源码位于 [standardization/JunctionToolkit](standardization/JunctionToolkit)，默认地图目录为 `standardization/maps`。`standardization/apply_standard.py` 已停用。

## 安装

获取仓库源码后，在根目录建立环境：

```powershell
py -3.12 -m venv generator-env
py -3.7 -m venv carla-env
& ./generator-env/Scripts/python.exe -m pip install -r standardization/JunctionToolkit/requirements-generator-lock.txt
& ./carla-env/Scripts/python.exe -m pip install -r standardization/JunctionToolkit/requirements-carla.txt
```

若无法通过 pip 获取对应平台的 CARLA 包，可安装 CARLA 0.9.15 发行包提供的、与 Python 3.7 和系统架构匹配的 wheel。

统一入口会自动查找仓库根目录的两个环境。使用其他安装位置时，新建 `standardization/JunctionToolkit/settings.local.json`，例如：

```json
{
  "generator_python": "../../generator-env/Scripts/python.exe",
  "carla_python": "../../carla-env/Scripts/python.exe",
  "workspace_root": "../..",
  "project": ""
}
```

相对路径以 `JunctionToolkit` 为基准；`project` 留空即可独立使用地图工具。它仅用于显式接入外部实验工程，不影响生成、美化与离线验证。

```powershell
& ./generator-env/Scripts/python.exe -B standardization/JunctionToolkit/toolkit.py doctor
```

`doctor` 检查两个 Python 环境及模块，不启动 CARLA 服务端。

## 在 CARLA 运行已有地图

先启动自己安装的 CARLA，例如：

```powershell
& 'D:/WindowsNoEditor/CarlaUE4.exe' -windowed -ResX=1280 -ResY=720 -quality-level=Epic -carla-rpc-port=2000
```

将示例路径改成实际安装位置。结束当前车辆、传感器和同步实验后，再加载地图：

```powershell
& ./generator-env/Scripts/python.exe -B standardization/JunctionToolkit/toolkit.py view standardization/maps/mixed_1x6
```

地图目录至少需要 `map.xodr`、`manifest.json` 和通过的 `topology_validation.json`，并应携带生成器输出的 `config.json`、`safety_rules.json`。单独一个 `.xodr` 不能替代完整工具包地图目录。

加载器自动选择默认美化，并在地图目录保存 `overview.png`、`street.png` 和 `load_report.json`；环岛专用加载器保存总览图。可用 `--no-photos` 跳过截图，用 `--host`、`--port` 指定服务器。加载已有地图不会自动生成车辆。

本地已生成地图位于 `standardization/maps`；GitHub 精简发布可能只包含部分示例。如果没有上述目录，先运行下一节的生成命令。

## 跑一个例子

下面生成采用新缺失规则的两车道纯三叉 1×6 路网，并完成离线验证。输出目录必须为空或不存在。

```powershell
& ./generator-env/Scripts/python.exe -B standardization/JunctionToolkit/toolkit.py random-t --config standardization/JunctionToolkit/configs/t_1x6.json --out standardization/maps/demo_t_1x6
```

CARLA 启动后加载、应用默认简化美化并截图：

```powershell
& ./generator-env/Scripts/python.exe -B standardization/JunctionToolkit/toolkit.py view standardization/maps/demo_t_1x6
```

生成阶段不要求 CARLA 服务端运行；统一入口会使用 CARLA Python 环境进行离线 waypoint 验证。加载与截图才需要连接服务端。

验证报告应显示 `passed: true`。离线通过代表入口出口、接缝与路口图等拓扑检查通过，不等价于实车通行、多车冲突或信号配时验证。用于实验替换前，还应运行实际车辆通行检查；接入主工程时可使用 `release_drive_check.py MAP_DIR --project PROJECT_ROOT`，该检查依赖主工程的预检模块，会切换地图并逐条测试转向和接缝。

## 配置

### 单路口

四叉和三叉使用 JSON 配置，通过以下入口生成：

```powershell
& ./generator-env/Scripts/python.exe -B standardization/JunctionToolkit/toolkit.py generate --config CONFIG.json --out standardization/maps/my_junction
```

单四叉最小配置：

```json
{"name":"single_cross","type":"cross","rows":1,"columns":1,"seed":42}
```

单三叉最小配置：

```json
{"name":"single_t","type":"t","rows":1,"columns":1,"missing_arm":"north","seed":42}
```

`missing_arm` 可为 `north`、`south`、`east`、`west`，单三叉默认缺北。方向采用地图方位：OpenDRIVE +X 为东、+Y 为北，CARLA 世界坐标的 Y 符号相反。

|类型|默认道路结构|入口从左到右的默认转向|
|---|---|---|
|四叉|每方向3车道，车道宽3.5m，路口半径22.5m|左转、直行、右转|
|三叉|每方向2车道，车道宽3.5m，路口半径19m|缺左转出口：直、右；缺直行出口：左、右；缺右转出口：左、直|
|环岛|四条接入道路，每方向2车道，双车道环道|按入口、环行和出口连接单独编码，不套用四叉的左/直/右分配|

三叉和四叉默认支路长度150m，人行道宽2m、路缘宽0.2m、高0.15m。模板约束由 `road_standard.py` 和 `templates/t_cross2/profile.json` 统一应用；不能仅修改 `incoming_lanes` 就把标准两车道三叉变成三车道。

单环岛使用独立入口，默认半径30m、支路长度120m、车道宽3.5m：

```powershell
& ./generator-env/Scripts/python.exe -B standardization/JunctionToolkit/toolkit.py roundabout --out standardization/maps/my_roundabout --radius 30 --arm-length 120 --lane-width 3.5
```

环岛包含多个 CARLA junction 控制节点，但在默认美化分类中仍算一个路口。加载后信号保持全红；附带的相位演示不代表已验证的安全通行配时。

### 路网

|需求|入口|默认车道结构|
|---|---|---|
|规则四叉网格|`generate`，配置 `type=cross` 和 rows/columns|每方向3车道|
|纯三叉随机路网|`random-t`|沿用两车道三叉模板|
|三叉/四叉混合随机路网|`random-grid`|所有入口统一每方向3车道|

混合路网配置示例：

```json
{
  "name": "mixed_3x3",
  "rows": 3,
  "columns": 3,
  "spacing_m": 344,
  "arm_length_m": 150,
  "seed": 42,
  "t_probability": 0.5,
  "roadside_props": false
}
```

```powershell
& ./generator-env/Scripts/python.exe -B standardization/JunctionToolkit/toolkit.py random-grid --config CONFIG.json --out standardization/maps/my_network
```

- `rows`、`columns` 为1～6；`spacing_m` 为相邻路口中心间距，`arm_length_m` 为边界支路长度。内部道路长度由间距及路口半径决定，并需满足生成器最小长度约束。
- `seed` 控制复现；`t_probability` 默认0.5，表示先以相同概率确定三叉/四叉类型，不保证数量严格相等。
- `junction_types` 可指定路口类型，例如 `{"0,0":"t","0,1":"cross"}`；坐标键为零起始的“行,列”。
- 缺失方向从四方向等概率提议，拒绝导致全网断开的布局。1×n中间路口只能缺北/南；n×1中间路口只能缺东/西。最终方向分布受连通约束，不保证各方向占25%。
- 先确定类型和缺失方向，再生成几何与 lane link。混合路网先生成全四叉网格，再删路并重建三叉内部连接；纯三叉直接使用现有两车道模板。
- 可通过 `missing_arms` 指定缺失方向。混合入口对其余三叉继续随机抽样；纯三叉入口使用固定方案时，未指定项采用 `missing_arm`（默认 north）。固定方案也必须通过连通检查。

三车道混合路网中的三叉，按驾驶员进道口从左到右编码：

|缺失的出口方向（相对当前入口）|左车道|中车道|右车道|
|---|---|---|---|
|左转|直行|直行|右转|
|直行|左转|左转|右转|
|右转|左转|左转|直行|

入口 -1/-2/-3 显式连接目标出口 +1/+2/+3，不依赖 CARLA 自动推断。两车道纯三叉使用前面的双车道规则，不使用双左转/双直行编码。

`random-grid` 遇到 `type=t`、`t_probability=1` 或所有路口显式指定为 t 时，会转入两车道纯三叉生成器。普通混合配置偶然抽到全三叉时，仍按混合路网的三车道模式生成。随机路网目前不提供“中心环岛混入网格”的已验证统一入口；单环岛使用独立生成器。

美化以外的配置变化通常需要重新生成地图。更新脚本不会自动改写已有 `.xodr`，应生成到新目录、验证后再替换。详见 [混合路网规则](standardization/JunctionToolkit/RANDOM_NETWORK.md)和[纯三叉规则](standardization/JunctionToolkit/PURE_T_NETWORK.md)。

### 美化配置

所有新地图复用统一加载器，不需要逐图编写美化脚本。

|`view --beautify`|行为|
|---|---|
|`auto`（默认）|单路口完整美化，多路口网络简化美化|
|`simple`|只新增停止线和信号杆，保留原生可控信号灯|
|`complete`|调用相应类型的完整美化流程|
|`off`|关闭运行时美化，仍保留原生道路与信号|

```powershell
& ./generator-env/Scripts/python.exe -B standardization/JunctionToolkit/toolkit.py view standardization/maps/demo_t_1x6 --beautify complete
```

单路口完整模式包含相应模板的路面标线、停止线、斑马线、信号杆，以及适用的草地、树木和人行道外观。网络完整模式恢复原有整路标线及斑马线等装饰，仍遵循网络禁用树木、草地网格和道路家具的标准。简化模式不删除 OpenDRIVE 已包含的原生道路、人行道或标线；运行时装饰关闭碰撞，原生道路物理保留。

外观参数入口为 [appearance_standard.json](standardization/JunctionToolkit/configs/appearance_standard.json)、[三叉外观模板](standardization/JunctionToolkit/templates/t_cross2/appearance.json)和[环岛外观模板](standardization/JunctionToolkit/templates/round_cross2/appearance.json)。配置中部分来源路径只记录参考资产，并不代表运行时会加载外部 RoadRunner 文件。

`load_report.json` 记录本次实际美化模式。详见[美化说明](standardization/JunctionToolkit/APPEARANCE.md)。

## 贡献入口

在 GitHub 仓库提交 Issue 或 Pull Request。问题报告请附最小配置、seed、复现命令、CARLA/Python版本、验证报告，以及必要的地图总览或近景截图。

|修改方向|主要入口|
|---|---|
|道路几何及标准参数|`generate.py`、`road_standard.py`、`templates/t_cross2/profile.json`|
|随机布局与连通约束|`network_plan.py`、`random_grid.py`、`random_t_network.py`、`template_network.py`|
|车道转向分配|`layout.py`、生成器中的 OpenDRIVE connection/lane link|
|加载与外观|`view.py`、`network_preview.py`、`network_simple_finish.py`、`runtime_finish.py`及类型专用模板|
|检查与实车验证|`validate.py`、`audit_experiment_map.py`、`release_drive_check.py`|

以上文件均位于 `standardization/JunctionToolkit`。修改拓扑时应同时检查配置、manifest、实际 OpenDRIVE 连接及实车表现；修改外观时应检查截图和装饰碰撞。示例测试：

```powershell
& ./generator-env/Scripts/python.exe -B -m unittest discover -s standardization/JunctionToolkit -p test_network_plan.py
& ./generator-env/Scripts/python.exe -B -m unittest discover -s standardization/JunctionToolkit -p test_pure_t_network.py
```

本工作区的 GitHub 发布副本位于 `github/DriveSignalLab/map_toolkit`，使用独立发布分支。发布前按最终仓库目录调整文档链接及命令路径，携带入口脚本的全部源码依赖、JSON配置、精选示例与报告。不要上传虚拟环境、机器专属 `settings.local.json` 或 CARLA/RoadRunner 商业资产二进制。向主工程部署地图或工具需另行明确授权。
