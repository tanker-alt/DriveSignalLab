> 已新增单个 T 型路口：向导类型选 `t`，或 `generate_map.cmd --type t --missing-arm north --name my_t`。详见 [T 型路口](T_JUNCTION.md)。原 cross/grid 用法保留。

# 交互生成入口

现在可在本目录运行 `generate_map.cmd`，按提示输入后自动保存配置并生成地图，无需复制修改 JSON。详细步骤见 [地图生成向导](WIZARD.md)。四方向独立进出口车道数为后续扩展，当前仍为四向同数。

# CARLA 路口生成工具包 v0.1

为 CARLA 0.9.15 生成可验证的四臂十字路口及同构规则网格；支持真实网格装饰和现有单路口评估项目。无需 Unreal 编辑器。主工程默认实验已切换到 Cross3，见 ../../README.md。

## 快速使用

在工具包目录打开 PowerShell。先启动 CARLA 0.9.15，等待上一轮实验完全结束。

```powershell
# 检查 Python 环境
.\toolkit.cmd doctor

# 已附带生成且验证过的示例，直接查看美化路口
.\toolkit.cmd view maps\cross3

# 原项目的 24 车完整实验；看到 Exit codes: 0 0 后才算结束
.\toolkit.cmd experiment maps\cross3 --vehicles-per-approach 6 --blueprint random --seed 29

# 查看四路口网格，再检查车辆连续穿过两个路口
.\toolkit.cmd view maps\grid2x2
.\toolkit.cmd drive-check maps\grid2x2
```

`toolkit.cmd` 使用 PATH 中的 python；可直接 python toolkit.py。settings.json 的 generator_python/carla_python 为空时使用当前解释器；跨环境分发时填本机两个解释器路径。project 相对于工具包目录，地图/配置参数相对于当前工作目录。正式流程先单独 view 加载地图，再分别调用原控制器与随机车辆生成器，见 ../../docs/guides/SETUP.md；run_experiment.py 只作可选自动验收。生成器用 Python 3.12.3 + scenariogeneration 0.16.7，仿真用 Python 3.7.9 + carla 0.9.15。

## 生成自己的地图

复制 `configs/cross3.json` 或 `configs/grid2x2.json` 后修改。运行：

```powershell
.\toolkit.cmd generate --config configs\cross3.json --out maps\my_cross
.\toolkit.cmd view maps\my_cross
.\toolkit.cmd experiment maps\my_cross
```

输出目录必须为空，避免覆盖旧地图和实验。`generate` 自动调用 CARLA 离线拓扑检查，失败返回非零退出码；失败候选不会自动加载。只生成不切换当前仿真。

| 配置 | 含义与范围 |
|---|---|
| `rows`, `columns` | 1–5，各行列交点生成一个四臂路口；总数为两者乘积 |
| `incoming_lanes` | 每个方向 1–4 条进口车道，出口车道数与进口相同 |
| `allowed_turns` | 每条进口车道允许的 `left`、`straight`、`right`，从最内侧到最外侧；所有方向共用 |
| `arm_length_m` | 外围道路臂长，80–500 米 |
| `spacing_m` | 相邻路口中心距离；每半条内部道路至少保留 80 米 |
| `lane_width_m` | 3–4.5 米 |
| `junction_radius_m` | 中心至道路端点距离，至少为半幅车行道宽度加 8 米 |
| `sidewalk_width_m` | 0–5 米，0 关闭人行道；其余生成高 0.15 米的人行道 |
| `roadside_props` | 是否生成人行道小设施；不生成建筑 |
| `seed` | 装饰资产选择的可重复随机种子；不是实验车队种子 |

例如 `rows=2, columns=3` 生成六个路口。内部道路端点真实连接，车道符号在 start-to-start 接口翻转；不是把孤立路口简单摆在一起。网格边缘保留通向地图边界的道路臂。

当前支持单个 T 字路口（见 T_JUNCTION.md），不支持环岛、各方向不同车道数、任意位置/角度路口或不同类型路口混搭。参数范围表示接受的配置，不表示每种组合已完成实车认证。多转向共享车道可生成，但任意转向组合不保证交通组织合理或冲突安全。

## 外观实现与边界

- 运行时装饰提供现有道路材质、地面、人行道、小设施和同向虚线网格，没有随机建筑。
- 进出方向之间是醒目的连续双黄 debug 叠加线，非烘焙标线、非物理障碍。RGB 预览不代表语义分割标注验收。
- 外观网格关闭碰撞，车辆道路物理来自 OpenDRIVE；重载需再次 view 恢复装饰。
- 人行道无行人导航网格；未实现精细路口转角、真实 PBR 城市场景和已校准停止线。
- 资产来自 CARLA 0.9.15 安装包，不复制 Unreal 资源。大网格性能需实测。

## 评估项目连接

`project_bridge.py` 在独立进程中适配现有 `Toolkits.CollectionTools.map_setup`、`control_junction_phases.py` 和 `spawn_junction_traffic.py`。主工程原始入口现已支持 profile 驱动的同类适配。每轮生成新 profile，检查实际世界的 XODR 指纹、信号 ID 及进口车道集合。

原生生成灯的 pole_index 均为 0，适配后日志 pole_index 表示设计相位槽位；默认 Cross3 为 0–11；实际灯态仍由原生 actor 控制。停止参考点取 signal_s−3 米，profile 保持 `reference_only`，不是人工标定停止线。

`experiment` 仅支持单路口，默认 12 辆 Model3，也支持上面 24 车混合示例；车队生成距离 65 米。输出写入 `maps/<名称>/experiments/<时间戳>/`，包括 logs、processes.json、profile 和 runs。等待 `Exit codes: 0 0`，并核对 metadata.status=completed。进程启动、异常和清理结果记录到 processes.json。

`drive-check` 是网格物理通行冒烟测试：每行一辆车，全部绿灯、只直行，通过这一行全部路口。它不是多路口联合信号控制、交通流评分或完整碰撞安全验收。

实验需要独占当前 CARLA。入口会拒绝已有同步模式或残留车辆/传感器。Ctrl+C/可捕获异常会执行清理；直接关闭窗口、强杀 Python 或 CARLA 崩溃不能保证 finally 执行。不要通过重启第二个控制器修复卡住的实验。记录日志后重启 CARLA、再 `view` 加载是明确的恢复路径。

## 输出与验证

- `map.xodr`：标准道路描述。
- `config.json`, `manifest.json`：完整参数、地图指纹、路口/车道/信号映射、预期转向和道路接缝。
- `topology_validation.json`：CARLA waypoint 的全部指定转向、方向及采样跳变检查，网格额外检查每条接缝车道。
- `load_report.json`：服务器地图/信号核对、装饰 actor 与失败清单。
- `overview.png`, `street.png`：真实 CARLA RGB 图像。
- `drive_validation.json`：网格实车测试结果。
- `VALIDATION.md`：本次实际验收范围；不能把离线验证当作实车验收。

生成环境：Python 3.12 / scenariogeneration 0.16.7；CARLA 客户端环境：Python 3.7.9 / carla 0.9.15；服务端 0.9.15。两个环境分开使用，本次未升级依赖。


## 每车道独立信号灯（2026-10-23 更新）

新生成地图默认一条进口车道对应一个原生 CARLA 信号灯。Cross3 是 12 灯、T3 是 9 灯；manifest.json 的 signals 给出 signal ID、road_id、incoming_lanes（单元素）和 phase_slot。外接控制器应按这份映射控制，不能写死 4 个灯。重载地图后 actor ID 会变化，需重新获取。

外观使用原生可控灯头和共用路边悬臂：立柱放在人行道侧，灯头悬在对应车道上方，隐藏原模型多余杆件。支架为关闭碰撞的视觉网格；原生灯态和车道触发器保留。必须通过工具的 view 加载才能恢复支架，不是新的打包 Unreal 资产。

操作仍是加载地图 → 外接控制器/固定相位采集 → 随机车辆。`--vehicles-per-approach 3` 仍指每个物理进口方向 3 辆，Cross3 总计 12 辆，T3 总计 9 辆。原固定控制器逐灯运行 12 秒绿灯 + 3 秒黄灯 + 2 秒全红，Cross3 每轮 204 仿真秒；示例采集改为 304 秒。该时间是基线测试长度，不保证任意车流均全部完成。

已有用户地图不会自动改写；需要用原 config.json 生成到新目录并重新准备其 profile。默认 cross3、t3_north 和 grid2x2 示例已更新；旧默认地图和配置保存在 archive/handoff_backups。默认 profile 仍标记 reference_only，实车停车测试不等同于物理停止线标定或全套评分验收。


## 官方杆架外观（2026-10-23）

view 现在复用 CARLA 安装包里的整套立柱、弧形悬臂和路灯网格：3–4 条进口车道使用 SM_TrafficLight01，1–2 条使用 SM_TrafficLight02；底座使用 SM_TrafficLight__Ground_base。灯头仍是逐车道原生可控信号，不是整套多灯头 actor 同步变色。无需改 JSON，重新 view 已有的逐车道地图即可应用；旧的一方向一灯地图仍需重新生成。

默认 Cross3 已进行近景接头检查和尺寸净空检查。灯头底约 4.98 m；四种大车型中最高包围盒的 Fusorosa 完成 12 条进口车道的受控直线穿越，最大车顶包围盒高度约 4.34 m，最小垂直余量约 0.63 m，立柱底座最小侧向余量约 0.40 m。结果见 maps/cross3/gantry_clearance_validation.json，近景见 signal_close.png。

杆架关闭碰撞；这里验证的是几何净空与受控通过，并非杆件物理碰撞认证，也不代表任意超高车、横向偏离或全部转向均已验收。本轮实车检查仅针对默认 Cross3。重载后需重新查询信号 actor ID，调用流程不变。
