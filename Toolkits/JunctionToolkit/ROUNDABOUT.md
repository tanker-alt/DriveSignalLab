# 单个环岛候选版

当前展示版：`maps/roundabout4_8groups_v4`，已在 CARLA 加载并核对 16 个原生灯的车道归属，A/B/C/D 无车灯态演示通过。操作见该目录 USAGE.md。当前显示 A 相位；这不代表交通冲突验收通过。

此前候选：`maps/roundabout4_2lane_signals_v3`。旧 v2 单车道版保留；v1 是早期急弯版本，不建议加载。下文的离线验收边界描述 v3，v4 的最新在线范围见 USAGE.md 与 phase_demo_validation.json。

## 已生成的结构

- 东、西、南、北四个接入口，每个方向 2 条进口 + 2 条出口。
- 环道全程保持 2 条车道，逆时针通行，内/外车道中心半径分别为 30/33.5 m；车道宽 3.5 m，外接直路长 120 m。
- 8 个入口信号 + 8 个环内信号，共 16 个原生信号，每个只控制一条车道。环内灯位于四个接入区上游，每处两灯。
- 内侧进口 → 内环 → 对应内侧出口；外侧进口 → 外环 → 对应外侧出口。环道内暂不设置自由换道连接，各车道均可绕环并到达四个方向的对应出口车道。
- 内侧车辆进出环道的轨迹会穿过外环车流，不能认为双车道天然无冲突。入口灯与环内灯的协调、冲突相位及环内排队控制均待设计，没有安装自动配时。

manifest.json 的 signals 包含 role=entry/circulating、road_id、incoming_lanes、junction_id、phase_slot 和 reference_s。入口信号 ID 为 1011/1012 至 1041/1042；环内信号为 20201/20202 至 20231/20232。这些是 OpenDRIVE ID，重载后应重新获取对应 actor。

CARLA 内部使用四个接入 junction（10、11、12、13）组成一个逻辑环岛。它不是原来单个 junction=10 的十字路口；不要直接套用默认 Cross3 profile 或原来的单路口评分入口。整体环岛采集/评分适配尚未完成。

## 生成与查看

在 junction_toolkit 目录下，使用现有配置好的环境：

```bat
D:\CarlaMapTools\generator-env\Scripts\python.exe -B toolkit.py roundabout --out maps\my_roundabout --radius 30 --arm-length 120 --lane-width 3.5
```

原型固定四臂、每方向 2 进 2 出、双车道环道以及 16 个逐车道信号；暂未加入交互向导。--radius 指内环车道中心半径。命令只生成文件并做离线检查，不切换 CARLA。

需要主动加载本次候选时，在 CARLA 已启动且没有实验运行的情况下执行：

```bat
D:\CarlaMapTools\generator-env\Scripts\python.exe -B toolkit.py view maps\roundabout4_2lane_signals_v3
```

这个命令会切换地图，但保留观察相机的原位置和朝向。加载器预留了逐车道官方杆架；默认把全部 16 个灯冻结为全红，等待用户的相位控制。全红只是初始保持状态，不是已设计好的配时。预览传感器不移动观察相机。

## 当前验收边界

已用 CARLA 0.9.15 的离线 Map 检查：20 条连接道路上的 24 条连接车道连续、16 个灯逐车道覆盖、环道全程为双车道且方向正确；8 条进口均能到达四个方向的对应出口车道，并能绕环循环。内外环均采样检查，最小转弯半径约 18.09 m。

`topology_preview.png` 是按 CARLA waypoint 绘制的几何预览，不是 CARLA 运行截图。当前会话没有切图，未做该环岛的实车、信号停车、路面网格或碰撞验收；加载器本身也尚未经本图在线验收。下一步应先做独占服务器的加载与逐路径单车验收，再决定相位与整体评估接入。
