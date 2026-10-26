# 单个 T 型路口

在工具包目录用 CMD 运行 `generate_map.cmd`，类型选择 `t`，再选择缺失方向 north/south/east/west。其余仍按提示输入，不需要编辑 JSON。

也可直接生成：

```bat
generate_map.cmd --type t --missing-arm north --lanes 3 --name my_t
D:\CarlaMapTools\generator-env\Scripts\python.exe -B toolkit.py view maps\my_t
```

已提供示例 `maps/t3_north`，查看时将上面 my_t 替换为 t3_north。不要覆盖已有名称。

## 交通组织

第一版仅单路口，3 个方向车道数一致、进出同数，支持 1–4 车道。没有通往缺失方向的道路、信号或连接。方向约定：east=OpenDRIVE +X，north=OpenDRIVE +Y（CARLA -Y）；俯视相机旋转可能改变屏幕上下方向。

各进口转向分别保存为 arm_turns。默认把剩余两个可达转向由内至外分配到全部车道，确保每条进口车道都有出口；1 车道共享两个转向。以缺北、3 车道为例：西进口直/直/右，南进口左/左/右，东进口左/左/直。不是简单删除原十字路口的一组连接后保留死车道。

交互模式可修改各进口允许转向，工具拒绝驶向缺失方向、空转向以及不能连接到两条剩余道路的配置。信号采取三个独立相位槽位；停止参考仍是 reference_only，不是校准停止线。

## 按原流程运行实验

先用 view 加载，再准备这个地图自己的配置（输出目录必须是新目录）：

```bat
D:\CarlaMapTools\carla-env\Scripts\python.exe -B project_bridge.py prepare --map-dir maps\my_t --junction-id 10 --prepare --output maps\my_t\profile
```

终端 A（工具包目录）：

```bat
D:\CarlaMapTools\carla-env\Scripts\python.exe -B project_bridge.py control --map-dir maps\my_t --profile maps\my_t\profile\profile.json --collect --collisions
```

看到 Frame collection output 后，终端 B（同目录）：

```bat
D:\CarlaMapTools\carla-env\Scripts\python.exe -B project_bridge.py spawn --map-dir maps\my_t --profile maps\my_t\profile\profile.json --vehicles-per-approach 3 --first-distance 65 --vehicle-blueprint random --duration-seconds 140
```

project_bridge 调用的仍是原控制器/车辆生成器，只适配新地图信号与车道。每方向 3 辆在 T 路口总共 9 辆。等 B 完成清理后在 A 按 Ctrl+C，数据写到原项目 evaluation/outputs。没有替换工程默认 Cross3 配置。

## 验证与边界

- 4 种缺失方向 × 1–4 车道，共 16 种配置离线转向、道路方向及信号检查通过。
- 原 Cross3 XODR 与修改前完全一致；规则 2×2 网格的 48 条转向、24 条接缝检查通过；原工程 87 项回归通过。
- 缺北三车道样例在 CARLA 0.9.15 中加载 3 组灯、各 3 进口车道；9 条指定车道转向逐一实车通行通过，0 记录碰撞回调。该检查是全绿灯、单车顺序测试，不代表并发冲突安全或全部配置已实车验收。
- 逐转向复查：先加载空闲地图，再运行 `python movement_check.py maps/my_t`（使用 CARLA Python）。`drive_check.py` 仍是四臂规则网格直行检查，不用于 T 路口。
- 缺失侧仍使用简化的路口铺装，不是精细封边/路缘石模型。没有随机建筑，环岛、T 字网格组合和各方向不同车道数尚未实现。

实际随机混合车队联调：9 辆车全部完成到达—驶出配对，0 截断、0 未闭合；5,140 帧、0 缺帧、0 信号不一致；控制器/生成器退出 0/0、恢复世界设置。run_id=20261023T054157Z_c14616e3，证据在 maps/t3_north/experiment_validation.json 与 experiments 子目录。此为该样例功能验收，不代表全部参数组合或并发交通安全认证。
