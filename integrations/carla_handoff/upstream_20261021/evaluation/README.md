# 逐帧采集说明与 Town05 历史验证记录

当前交付入口请读 [安装与运行](../docs/SETUP.md)、[可执行契约](../docs/CONTRACT.md) 和
[地图工具](../map_tools/README.md)。本文件保留历史实验说明，旧字段/命令不作为最新接口依据。

## 2026-10-21 当前入口

协作开发请先读 [接口契约](../docs/INTEGRATION.md) 和 [api.py](api.py)。
默认实时读取当前地图及指定路口，生成本次检测线；不再默认使用 codex 旧 inventory。
测试车辆默认 random，多车型；通用阈值位于 configs/metrics.json。
无信号路口原始采集可用，但等待统计仍依赖唯一信号关联；跨地图尚待现场验收。
当前 schema 1.2 包含包围盒、控制量和相位字段，sample 返回同帧真值字典；完整字段见 CONTRACT.md。
回放现在使用实验内 detection_lines_used.json，缺失快照时不计算流量，不套用当前旧配置。

```powershell
python -m evaluation.run_collection --list-junctions
python -m evaluation.run_collection --junction-id 1427 --frames 200
python evaluation/topology.py --junction-id 1427 --output evaluation/configs/new_calibration.json
```

以下保留 10 月 20 日的字段语义和 Town05 操作/验证历史；其中固定车型、默认文件、
回放读取当前配置、无参数 topology 命令等旧行为已由以上更新替代。

当前阶段采集真值，并计算车辆事件和进口道停车等待；暂不判断闯红灯或评分。默认读取已经导出的
`../codex/junction_signal_inventory.json`，用 OpenDRIVE ID 匹配当前信号 actor。
无需再次导出；若地图或停止线校准发生变化，请同步更新 inventory。

## 推荐：与相位控制器一起运行

先启动 CARLA 并加载 Town05。使用能 `import carla` 的 Python，在
`D:\WindowsNoEditor\CarlaUE4\newProject` 打开终端：

```powershell
python control_junction_phases.py --collect
```

另开终端生成车辆（也可以用你自己的车辆程序）：

```powershell
python spawn_junction_traffic.py --vehicles-per-approach 1
```

控制器启用同步模式，默认步长 0.05 仿真秒；每次 tick 后完成采集、刷新文件，
再继续推进。开始生成车辆前的空场景也会记录。Ctrl+C 结束；也可用 `--cycles 2`
自动结束两个周期。控制器恢复原世界设置；车辆生成程序仍需自行停止以销毁车辆。
不要同时运行其他调用 `world.tick()` 的程序（包括原停止线可视化脚本），
也不要让其他程序同时切换这些信号。现在只将目标路口各灯的自动配时临时延长，
由控制器手动切灯，退出时恢复原配时和灯色（不恢复精确的已用相位时间）。
其他路口继续自动运行，不再全图冻结或全图设红。

若旧版运行异常结束，遗留全图冻结，首次恢复时运行：

```powershell
python control_junction_phases.py --collect --reset-all-signals
```

这个显式选项会解除全图冻结并重置信号周期；后续正常运行不需要添加。
车辆程序按一次 Ctrl+C 即请求退出，等待当前 RPC 返回后批量清理，清理期间
重复 Ctrl+C 被忽略。看到 `Cleanup verified` 才表示已核验本次车辆全部移除；
服务器不可达会输出未核验 ID。建议先停止车辆程序、看到清理结果，再停止控制器。
默认不逐车打印调试采样，必要时加 `--verbose`。

可选参数：`--collection-radius 120`（米）、`--fixed-delta-seconds 0.05`、
`--collection-output evaluation/outputs`、`--inventory <JSON路径>`。

## 独立监听（调试用）

```powershell
python evaluation/run_collection.py --frames 200
```

它不推进世界。同步模式下必须另有推进者；异步或独立监听可能漏帧，不能把
“每个收到的快照都记录”理解为服务器每帧都已记录。超时会保留已采集数据并退出。

## 输出与字段约定

每次运行创建唯一目录 `evaluation/outputs/<UTC时间_随机后缀>/`：

| 文件 | 内容 |
| --- | --- |
| metadata.json | 地图、步长、采集范围、完整拓扑副本、结束状态、帧统计 |
| frames.csv | 每帧一行；即使没有车辆，也记录车辆数和信号数 |
| vehicles.csv | 每帧每车一行；位置、姿态、速度、加速度、当前车道与关联信号 |
| signals.csv | 每帧每灯一行；运行时 actor ID、OpenDRIVE ID、灯色、冻结状态 |

共同主键为 `run_id + frame`，车辆加 `vehicle_id`，信号加 `signal_opendrive_id`。
`sim_time` 是服务器仿真秒，不是现实时间；位置单位米，速度 m/s，加速度 m/s²，
旋转单位度。速度、加速度模长是三维模长，并非有符号纵向分量。
车辆位置、姿态、速度、加速度全部从该帧 ActorSnapshot 读取。

- 采集区域是所有停止线参考点中心周围默认 120 米的水平圆形区域，包含区域内所有车辆。
  `junction_id` 表示本次评估目标，`current_junction_id` 才是车辆实际所在路口。
- 当前车道每帧重新查询；路外未匹配时字段留空，不复用生成时车道。
- 信号关联从停止线沿车道向上游追溯，不穿过相邻路口。`signal_match` 为
  `unique` / `ambiguous` / `unmapped`。多候选时保留候选列表，不强行给出灯色。
  车辆进入路口后可能不再匹配进口道信号，这是预期行为；本阶段尚未引入路线追踪。
- `stop_plane_front_distance_m` 仅在车辆与停止线同 road/section/lane 且停止线唯一时输出。
  它是车身包围盒最前端到参考停止线平面的水平投影距离，线前正、越线负。
  **不是沿弯曲道路的路径距离，也不能单独作为正式闯红灯判据。**
- 空值表示未知/不适用，不能按 0 处理。剩余相位时间和下一相位尚未输出。
- `frame_gap` 是与上一条记录之间丢失的帧数；`read_overlapped_tick` 表示采集期间
  世界是否已经推进。严格集成模式下二者应分别为 0、False。
- 信号状态不在 ActorSnapshot 中，依赖两次 tick 之间读取 actor 缓存。
  若有其他信号控制者，即使帧号未变，也无法保证灯色一致性。
- 每帧刷新 CSV；异常退出已刷新数据仍可读取。强杀进程时 metadata 可能停在
  `running`，此时应以 CSV 实际内容为准，不代表实验完整结束。

## 首次验收

1. 空场景启动采集，确认 frames.csv 持续增长，车辆数为 0。
2. 生成车辆，确认车辆当前车道和位置随帧变化；每帧有四条信号记录。
3. 观察红灯停车、绿灯通行；比对灯色及局部停止线距离与场景画面。
4. Ctrl+C 后检查 metadata 的 missing_frames 为 0，frames 中无采集跨帧。

无服务器验证：在本目录上级运行 `python -m unittest discover -s evaluation/tests -v`。
这些测试验证记录与几何逻辑，不替代 Town05 上的实际运行验收。

## 第二阶段：事件追踪与等待统计

新增文件按最初职责拆分：`vehicle_tracker.py` 管理车辆状态，`metrics.py` 计算
车道统计，`recorder.py` 持续写出派生结果，`summarize.py` 回放旧实验。
`collector.py` 已接入同一套逻辑，原 `--collect` 命令无需变化。

参数统一放在 `configs/town05_j1427.json`：水平速度不超过 0.1 m/s 时进入等待，
达到 0.3 m/s 时退出等待；中间区间保持原状态，避免反复启停。
首次观测后的 1 秒不统计等待，用于排除生成稳定期。这是“观测预热”，
并非已知出生时间；已有车辆首次进入采集范围也会排除 1 秒。

等待只发生在目标信号唯一关联的进口道，且车辆尚未进入目标 junction。
不依赖灯色，也不依赖停止线距离是否为正，因此包括绿灯排队停车，并避免
参考平面偏差把正常停止位置排除。缺帧、消失或实验结束时截断已观测等待，
不补计未知时间；不把一次消失直接判为通行完成。使用区间两端均符合条件的
保守积分，每次状态边界可能少计约一个采样间隔。

新增输出：

- `events.jsonl`：首次观测、进口道首次观测、等待开始/结束、参考平面过线、
  进入/离开目标路口、观测结束。进口道首次观测不等于上游检测线到达事件。
- `lanes.csv`：每帧非空进口车道的车辆数、停车车辆数、平均水平速度、当前车辆
  本次通行累计等待的均值。空车道省略；停车数不等于经过队尾连续性判定的队列长度。
- `summary.json`：每次 visit 的等待秒数、停车次数、路口进入/离开时间、完成状态。
  `last_observed_time` 可能远晚于 `junction_exit_time`，不能将其相减作为通行时间。

当前完成判据为 **actor 原点从目标 junction 转入普通道路**；不是车尾完全清空，
也不是独立校准的出口线。未来确定出口检测线后应升级版本，不混用统计定义。
`reference_plane_crossing` 始终不判定违规，`is_violation=null`。
车辆在下游路口等待期间继续保留原始轨迹，但不累计目标路口等待。

旧数据回放命令（在 newProject 目录执行，不需要 CARLA）：

```powershell
python evaluation/summarize.py evaluation/outputs/20261020T061415Z_8e0ac7b4
```

结果写入原实验目录下新的 `analysis_<时间_随机后缀>`，不覆盖原数据。
回放的 `status=completed` 表示分析结束，不表示源实验一定正常结束或没有截断。

## 第三阶段：实验条件、碰撞和检测线

生成器默认使用 `vehicle.tesla.model3`，由 TM 随机选择转向。
这是一段转向指令，不是完整路径规划；进口车道必须支持所选动作，实际执行
需核对轨迹。默认变更是为了建立一致基线，旧的混合车型/随机转向可显式选择。

```powershell
python control_junction_phases.py --collect --collisions
python spawn_junction_traffic.py --vehicles-per-approach 6 --vehicle-blueprint vehicle.tesla.model3 --route random --seed 17 --duration-seconds 180
```

两条命令分别在两个终端运行。`--route` 支持 Straight、Left、Right、random；
`--duration-seconds 180` 表示生成完成后运行 180 仿真秒，再自动清理本次车辆。
控制器应保持运行直到车辆程序报告清理完成；否则 TM 所属进程退出可能导致客户端连接报错。
`--vehicle-blueprint random --route random` 恢复混合测试模式。
每次生成器在脚本目录写唯一 `spawn_manifest_*.json`，记录种子、车型、请求动作、
生成车道、实际生成距离和车辆 ID。固定种子不能代替相同的同步时序、初始场景与路线核验。

碰撞传感器由采集器为首次进入采集范围的车辆安装，直到实验结束清理。
`collisions.jsonl` 中原样保留事件帧号、对象 ID、冲量和传感器安装记录。
安装前的碰撞未知；回调可能晚到，文件关闭时尚在传输的回调也可能缺失。
车辆离开采集半径后传感器仍在监听，可通过同帧车辆记录判断事件是否在评估区域。
一对车可能各报告一次，同一接触可能多次回调，**不能直接将行数当作事故数**。

`configs/detection_lines.json` 已从现有拓扑生成停止参考线候选，默认全部禁用。
现场确认后再设置 enabled=true；arrival/exit 检测线应按实际道路位置补充，
不要在弯路上简单平移停止线来伪造已校准检测点。
center_xy 是世界坐标中心，normal_xy 指向计数行驶方向，half_width_m 是半线长。
`detection_lines.py` 对连续帧车辆原点运动线段求交，只报告正向、线段范围内过线。
在 `events.jsonl` 输出 detection_line_crossing，保留线 ID、类型与插值时间。
它与原车头参考平面事件使用不同参考点，不能混作闯红灯判据。
原始到达/驶出边界统计暂不替换；校准完成后再基于检测线扩展滚动流量指标。

## 2026-10-20 更新：已启用进口/出口流量断面

以上第三阶段的“候选线默认禁用”描述的是原停止参考线；已保留至
`configs/detection_lines_stop_reference.json`。当前 `detection_lines.json` 是 16 条
启用的流量线：每个方向两条进口、两条出口，共 8 arrival + 8 exit。
到达线由停止参考 waypoint 沿车道向上游追溯 10 m；出口线沿 junction 出口
向下游追溯 5 m。位置、宽度、方向取自 CARLA 地图，已生成俯视图并经轨迹回放。
这些是自定义计数断面，不是道路真实停止标线的标定结果。

```powershell
# 需连接 Town05；重新生成配置、俯视图，并在场景中绘制 300 秒，不主动 tick
python evaluation/topology.py
```

可视化文件：`configs/detection_lines.png`。同一车道的重复出口 waypoint 合并，
有分歧时拒绝自动生成。检测同时检查相交方向、有限线宽和 road/section/lane。

`flows.csv` 每帧每条启用线一行，包括零流量：

- cumulative_count：本次观察到的正向过线次数；同车绕行回来再次计数。
- unique_vehicle_count：该线累计不同车辆 ID 数，不能把各线此列相加当作总车辆数。
- window_count：最近 60 仿真秒的过线次数，窗口为 (t-60,t]。
- nominal_rate_veh_per_hour：window_count×60，是窗口等效小时流率，并非整小时实测量。
  启动不足 60 秒时窗口尚未完整覆盖，使用者需结合 frames.csv 判断预热阶段。

出口计数表示车辆原点越过出口断面；它与旧 junction_exit 的拓扑边界不同，
两个指标明确分开保留。出生在到达线以内的车辆不补造到达，缺帧不插值跨帧过线。
`detection_lines_used.json` 在实验开始即保存配置快照；summary 保存 flow_totals。
回放使用当前配置，可用于重新校准后的比较，并在分析目录保存所用快照；
不是自动复现原实验的旧检测线定义。
