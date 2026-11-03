# 单路口信号控制交付

从本地已运行版本提取，CARLA 客户端/服务端均使用 **0.9.15**，Python 3.7。
源码来源和包装差异见 `SOURCE.json`。未重写控制逻辑；只提取两个共享辅助函数，解除对采集和评估工程的依赖。

## 获取与运行

```bash
git clone --branch feat/single-junction-signal-controller https://github.com/tanker-alt/DriveSignalLab.git
cd DriveSignalLab
python -m pip install -r signal_control/requirements.txt
# 从独立 maps 分支取得地图与路线适配器；也可合并该分支。
git fetch origin maps
git restore --source=origin/maps -- Maps
python -m unittest discover -s signal_control -p test_t_phases.py -v
python signal_control/control_t_phases.py --map-dir Maps/MapDataset/t3_north --plan-only
```

`fixtures/` 仅含单元测试所用 manifest，不是可加载地图。
CARLA 已启动、当前实验结束后加载地图，再执行控制：

```bash
python Maps/route_adapter/adapter.py load --map-dir Maps/MapDataset/t3_north
python signal_control/control_t_phases.py --map-dir Maps/MapDataset/t3_north --cycles 1
# 四叉：先加载 cross3；该入口只接受一个四叉路口。
python signal_control/control_four_phases.py run --map-dir Maps/MapDataset/cross3 --cycles 1
# 环岛：先加载 roundabout4_8groups；配时来自 phase_example.json。
python signal_control/control_roundabout_phases.py --map-dir Maps/MapDataset/roundabout4_8groups --cycles 1
```

加载器只加载原生 OpenDRIVE，不添加美化停止线。上面三条控制命令分别要求已加载对应地图。

## 给路网适配器开发者

|控制器|可复用接口|时钟和范围|
|---|---|---|
|三叉 `control_t_phases.py`|`build_plan(design, junction_id)`、`expected_states(plan, phase_index, stage)`、`bind(world, plan)`、`run(args, plan, sample_callback=None, client=None)`|选择一个路口；默认等外部帧，只有 `--tick` 主动推进|
|四叉 `control_four_phases.py`|`normalize_design(design)`、`expected_states(design, phase, stage, right_turn)`、`bind(world, design)`|仅单四叉；默认等帧，`--tick` 为唯一推进者|
|环岛 `control_roundabout_phases.py`|`load_plan(map_dir, path=None)`、`states(design, phase, stage)`|当前运行器独占同步世界并逐帧 tick，不能直接与其他运行器并发|

这些是各自现有 API，不是统一非阻塞控制器接口。路网协调层应复用计划和灯色函数，统一拥有 `world.tick()`，并移植每路口清空互锁和读回验证；不要启动多个阻塞 `run()` 后让它们各自 tick。

三叉回调签名为 `sample_callback(snapshot, phase_index, stage, expected, actual)`；不能在回调中推进世界。
跨回合使用同一客户端与 Traffic Manager 实例，初始化 TM seed 必须在接管信号之前完成。
灯色映射的 key 是 **OpenDRIVE signal ID 字符串**，不是 CARLA actor ID。地图以原始 `map.xodr` SHA256 绑定。

三叉每个进口最右侧车道保持绿色（可能是直行，并不只限右转）。`MAIN_RED` 只将受控车道置红，常绿车道仍通行；`ALL_RED` 才是全部红灯。
受控车辆未清空则等待，超时退出，不强制放行下一进口。常绿策略是本项目特定设计，不应直接推广为任意路口安全策略。

## 验证证据与边界

- 本次发布副本通过 4 个现有离线测试：相位规则、四方向旋转、路网隔离及非法配置拒绝。
- `evidence/t_single_20261103.json`：本地 2026-11-03 六车单三叉测试通过，6/6 完成、碰撞回调 0、车辆/传感器清理完成。
- `evidence/mixed_t_audit_20261103.json`：本地混合路网选中一个三叉路口，27 辆车、2202 帧，碰撞回调/红灯越线/错误出口均为 0。
- 历史四方向混合车流矩阵为 23/24 通过；最新单例通过不代表失败矩阵已全部修复，见 `docs/T_NETWORK_VALIDATION_2026-11-02.md`。
- 四叉、环岛脚本一并作为现有实现交付，不将三叉的验收数据套用到它们。环岛计划不是已认证的路网冲突安全方案。
- 本次包装仅做离线回归，没有重新接管 CARLA 或重跑动态实验。美化停止线与正式车头越线指标仍待统一标定。

`validate_t_phases_live.py`、`validate_t_network_live.py` 是动态复现入口；运行前先结束其他实验并加载匹配地图。详细说明位于 `docs/`。
