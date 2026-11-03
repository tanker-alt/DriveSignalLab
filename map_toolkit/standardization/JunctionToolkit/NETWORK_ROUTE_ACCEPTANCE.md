# 3×3 连续跨路口驾驶验收（2026-11-03）

验收地图：`standardization/maps/cross3x3_continuous_20261103`。

## 已验证结果

- 9 个四叉路口，108 项路口转向检查、72 项接缝方向/车道检查全部通过。
- 同一辆 Tesla Model 3（本次 actor 224）连续经过全部 9 个路口及 8 处接缝，完成约 3001.98 米路线。
- 4 次在虚线区完成的同向换道；零碰撞、零重生、零传送；车辆保持物理仿真。
- 实际仿真时间 473.4 秒，最大离散路径跟踪误差 0.653 米，到达判定时距终点 1.857 米。
- 单车、全部信号灯冻结为绿灯，使用路径跟踪控制器。没有使用 Traffic Manager 或原项目的规划器。这是跨路口连通及物理驾驶验收，不是信号控制/多车交通验收。
- 按车辆实际位置查询 CARLA waypoint 记录经过的路口；接缝验收来自实际车辆道路 ID 的相邻变化，不以计划路径代替车辆轨迹。
- 车辆到终点后停车；`--keep-vehicle` 保留车辆，spectator 保留跟车视角。

路线顺序（OpenDRIVE 行号从南向北）：

`10 → 10010 → 20010 → 50010 → 40010 → 30010 → 60010 → 70010 → 80010`

起点：road 1 / lane -2 / s=25。终点：road 80003 / lane +2 / s≈65.18。
起终点以带方向的 OpenDRIVE 车道位置指定，避免只按路口中心坐标定位。

## 连接语义与本次修正

核对本机 scenariogeneration 0.16.7 的 `xodr/links.py::_create_links_roads`：两条参考线 start-to-start 连接时，两侧都使用 predecessor，车道编号反号。predecessor/successor 描述参考线 s 方向，不等同于车辆行驶方向。

撤回之前在接缝处错误添加的 lane successor；道路另一端的 successor 继续指向本路口。当前道路接缝可以由 CARLA `Waypoint.next()` 双向穿过。新生成地图没有复现“接缝缺少连接”的假设；原先车辆消失的根因尚不能由本次结果认定。

每条入口车道只允许指定转向，因此固定车道的 `next()` 路径不能保证遍历全部路口。新增 `network_route_check.py` 使用实际 CARLA waypoint 连续通过路口/接缝，在进路口前按虚线换道权限逐条换到目标转向车道。不存在跨接缝坐标插值或传送；仅合法换道段采用平滑横向轨迹。

额外负向检查：在内存中删除 road 3 的接缝 predecessor 后，路线规划明确失败；从实际记录中删掉 road 10001 后，接缝验收明确失败。没有修改已验收地图来运行这两项检查。

## 复跑

在工作区根目录运行。生成器使用 Python 3.12 环境，CARLA 检查使用 Python 3.7 环境。输出目录必须为空，复跑生成请另取新目录名。

```powershell
& ./generator-env/Scripts/python.exe -B standardization/JunctionToolkit/generate.py --config standardization/config_cross_3x3.json --out standardization/maps/cross3x3_new_trial
& ./carla-env/Scripts/python.exe -B standardization/JunctionToolkit/validate.py standardization/maps/cross3x3_new_trial
& ./carla-env/Scripts/python.exe -u -B standardization/JunctionToolkit/network_route_check.py standardization/maps/cross3x3_new_trial --port 2002 --realtime --keep-vehicle
```

先启动一个可见 CARLA 实例，等待初始化稳定。成功验收时使用：

```powershell
Start-Process -FilePath 'D:/WindowsNoEditor/CarlaUE4.exe' -ArgumentList '-windowed','-ResX=1280','-ResY=720','-quality-level=Epic','-carla-rpc-port=2002' -WorkingDirectory 'D:/WindowsNoEditor' -WindowStyle Normal
```

`--realtime` 按仿真时间播放，约 8 分钟；省略则尽快执行，跟车视角仍然更新。`--plan-only` 只生成路线。脚本拒绝覆盖有车辆或同步实验的服务，因此先结束之前保留车辆的会话，再进行复跑。

本次早期加载遇到引擎崩溃，通过本机 PDB 将调用栈定位到 `FLandscapeRenderSystem::FGetSectionLODBiasesTask`，加载自带 Town01 也出现异常。DX11、单线程等尝试并未解决；最终在单实例 Epic 模式、初始化稳定后完成验收。尚未证明其中哪一个因素单独消除了异常，不把此异常归因于路网拓扑，也不修改 CARLA 安装或图形驱动。

## 交付文件

地图目录中的 `continuous_route.json` 保存路线、起终点、转向和换道段；`continuous_drive_validation.json` 保存实车验收结果和实际经过的 8 处接缝；`continuous_trajectory.json` 保存实车采样位置；`follow_camera/` 保存起点、9 个路口和终点共 11 张跟车图像。

`topology_validation.json` 与实车报告均记录最终地图 SHA256：
`9486cd1b265e703cfde8eddc56573de35971902f97dc09c2121a20085cebe2eb`。

所有交付保留在 CarlaMapTools；没有复制至 CoDriveSignal 或发布。
