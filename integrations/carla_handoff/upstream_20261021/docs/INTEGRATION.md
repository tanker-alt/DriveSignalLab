# 协作入口与采集契约（2026-10-21）

环境和最短运行以 [SETUP.md](SETUP.md) 为准；当前可执行字段/版本以 [CONTRACT.md](CONTRACT.md) 为准。
本文下方消息契约是后续协作设计；未实现部分不属于当前交付承诺。

## 新增派生接口

### 信号控制与 TM 初始化约束（已修复并在线验证）

CARLA 0.9.15 的 TrafficManager.set_random_device_seed 会重置全图红绿灯。
只能由世界主程序在信号接管和采集之前调用，不能在车辆适配器/生成器运行期间调用。
固定控制器新增 --tm-seed（默认 17）；生成器 --seed 仅控制本地车型/顺序随机，清单标注 seed_scope。
启动阶段设置种子会重置其他路口的自动周期；接管之后仅写目标路口，不进行全图冻结。
多路口协作应由唯一世界主程序一次性初始化 TM，不能由每个路口分别初始化种子。

每次写灯后执行不推进 tick 的同步 RPC 屏障，下一帧读取实际灯色并验证。
phases.csv 增加 signal_consistency（matched/mismatch）及 signal_mismatch_ids；
signal_checks.jsonl 保存逐帧期望/实际灯色；controller_config.json 保存 TM 种子和校验策略。
若不一致，保存异常帧后报错结束，metadata 状态 failed；实际灯色不被计划值覆盖。
旧低频相位日志字段改名 pole_states_before_tick，明确其为推进前缓存，不能当成新相位执行回执。
完整验证见 [相位修复报告](PHASE_FIX_2026-10-21.md)。

sample 返回包新增 vehicle_context 列表：每车的进口几何匹配、原点/车头/车尾参考距离、
参考来源与校准 ID、入口信号 ID/关联来源/同帧灯色。未匹配字段为空，不补零。
retained_entry_signal 仅表示连续观测保留的历史入口信号，不保证为路线级当前控制灯。
原始 vehicles 字段不被该派生关联覆盖。

create_collector 增加可选 stop_reference_config 字典；命令行支持 --stop-reference-config JSON路径。
完整格式、范围限制、双等待均值及 trips.jsonl 契约见 [METRICS.md](METRICS.md)。

明天（10 月 22 日）接入开发先读本文和 `evaluation/api.py`。接口版本为 0.1，
原始 CSV schema 为 1.2；这是可运行的采集边界，智能体推理和动作适配尚未实现。

## 最短接入方式

### 与部署无关的智能体消息契约（草案，尚未实现）

用户已确认先定义接口，不预设同进程、子进程或远程服务。以下为协作设计草案，
不代表 collector 已接受这些消息，也不改变现有 sample API。
消息采用可 JSON 序列化的数据；不跨边界传递 world、actor、snapshot 等 CARLA 对象。
同进程函数与未来传输适配器使用同一消息内容，只有主循环可执行动作和 tick。

| 消息 | 必需信息 | 语义 |
| --- | --- | --- |
| SessionContext | protocol_version、session_id、map_name、opendrive_sha256、fixed_delta_seconds、junction_ids | 主循环创建的会话身份；多个采集器可有各自 run_id。地图重载必须新建会话。 |
| Observation | session_id、observation_id、frame、sim_time、payload | 第 t 帧可用观测；payload 由适配器按模型权限构造，不默认把评估真值送给模型。 |
| ActionRequest | session_id、request_id、observation_id、based_on_frame、target_frame、actor_type、actor_id、action_type、payload | 声明依据哪帧、希望作用于哪帧；车辆/信号动作内容随双方能力协商。 |
| RouteIntent | session_id、vehicle_id、based_on_frame、valid_until_frame、junction_id、route_revision、path、maneuver | path 使用有序 road_id/section_id/lane_id/s 路径点；maneuver 可为 left/straight/right/u_turn/unknown。未知路线显式为空。 |
| ActionResult | session_id、request_id、status、reason、applied_before_frame、effective_action | 返回 applied/rejected/expired/failed；未执行时 applied_before_frame 为空。信号请求与安全过渡后的实际动作需区分。 |

主循环时序：tick 得到 t → sample/准备观测 → 请求决策 → 校验并应用动作 → 下一次 tick。
用户已确认先采用逐步模式，仿真等待决策。target_frame 为 t+1；远程延迟不能把过期动作悄悄改投更晚帧。
主循环配置 CARLA 同步模式及固定仿真步长；模型等待期间不调用 tick。
本步所需决策收齐、校验及动作应用完成后才推进，其他客户端不得推进世界。
同一车辆或信号的动作执行者必须唯一；不同智能体的决策频率可另行配置，
不要求信号智能体每个物理 tick 都重新选择相位。
推理耗时采用墙钟单独记录；等待、通行时间和相位倒计时采用仿真时间，
不把模型计算的墙钟等待计入交通指标。这一模式不用于证明模型具备实时推理能力。
决策缺失或失败时不自动 tick、不静默沿用上一动作；墙钟超时后的重试/终止策略待确定，
并保留人工中断与清理入口。以上为待接入主循环的契约，现有采集器不负责调度模型。
执行端校验会话、车辆/路口归属、观测引用和帧窗口；request_id 幂等，重复消息不重复执行。
多个动作源冲突须显式处理，不能依靠消息到达顺序覆盖。applied 只表示调用执行成功，
不保证车辆已达到目标状态，物理结果仍从后续观测判断。

路线关联采用已确认的“路线＋坐标”：在有效帧窗口内结合地图路径筛选候选信号。
无路线、路线过期或仍多候选时保留候选与未知；转向标签本身未必能唯一确定灯。
历史入口灯关联独立保留来源，不伪装成路线级匹配。

尚待讨论：动作 payload（控制量/目标速度/轨迹，选相位/逐灯状态）、
墙钟超时期限与重试/终止策略、黄灯/全红执行职责。
这些参数未确定前，草案不自动启用任何驾驶或信号回退动作。

由外部主程序先加载地图并取得 world，然后创建采集器。采集器始终读取此 world 的地图，
不会替用户加载 Town05。RoadSpec 是用户输入道路信息的接口，junction_id 是当前地图的路口 ID。

```python
from evaluation.api import RoadSpec, create_collector

roads = RoadSpec(junction_id=1427, radius_m=120, expected_map="Town05")
collector = create_collector(world, roads)
status = "failed"
try:
    while running:
        # 主程序在这里应用上一帧决策产生的车辆/信号动作。
        # 仅主程序拥有 tick；同步模式、固定步长和 TM 设置也由它管理。
        world.tick()
        packet = collector.sample(world.get_snapshot())
        # 用 packet 更新评估或适配器，再准备下一帧动作。
    status = "completed"
except KeyboardInterrupt:
    status = "interrupted"
finally:
    collector.close(status)
```

示例中的 world、running 和动作循环由接入方提供。正式实验必须在两个 tick 之间完成 sample，
并保证信号在本次读取期间不被其他线程/进程修改。重新加载地图后关闭旧采集器并重新创建。

## 输入与输出

### 道路输入：自动生成＋用户覆盖（用户已确认）

目标流程：读取当前加载地图及 OpenDRIVE 指纹 → 用户选择 junction_id 与目标检测长度 →
自动生成进口路径、参考断面和检测区域 → 报告歧义/截短 → 用户按需覆盖 → 保存本次实际配置快照。
用户提供的有效覆盖优先；未覆盖部分自动生成。无法解析或身份不匹配应明确报错，
不能静默忽略覆盖、借用其他地图配置或把无法计算的指标填成零。

| 内容 | 已有能力 | 待补接口 |
| --- | --- | --- |
| 地图与路口 | 当前 world、路口发现、RoadSpec.junction_id、地图指纹 | 跨地图现场验收 |
| 目标检测长度 | approach_metrics.json 的 detection_length_m，默认 50 m | 公共入口按次传入，并支持不同进口设置 |
| 到达/出口断面 | 自动生成；detection_config 整套替换 | 按进口局部覆盖与统一校验 |
| 停止参考点 | stop_reference_config 按进口覆盖同 road/lane/section 内的 s | 更一般的用户参考断面 |
| 进口路径 | 根据停止参考 waypoint 自动向上游采样，遇分叉截短 | 用户指定有序路径、无信号进口路径 |
| 检测区域 | RoadSpec.radius_m 的水平采集圆；派生进口条带 | 用户区域边界、立交高度及路网归属限制 |

这是已确认的设计方向，不能把表中待补接口当成当前可传参数。
现有 detection_config 仍是整套检测线替换，不会自动合并未提供的线。
每个进口应保留目标长度、实际长度、截短原因与配置来源；检测区不能为了达到目标长度
自动跨入相邻路口。用户已确认且已实现：新实验自动到达线与各进口实际检测区上游边界对齐。
显式 detection_config 仍优先且整套替换；RoadSpec.arrival_distance_m 指定数值可覆盖对齐策略。
无信号路口尚无进口区域几何，默认对齐会明确报错，需要显式距离或检测线配置。

| 接口 | 约定 |
| --- | --- |
| RoadSpec.junction_id | 必填，地图内路口 ID；没有“通用的 1427” |
| radius_m | 采集区域水平圆半径，默认 120 m；中心优先使用停止参考点均值，无停止点时使用路口包围盒中心 |
| expected_map | 可选的地图名称检查；不触发加载；metadata 还保存 OpenDRIVE SHA256 |
| arrival_distance_m / exit_distance_m | 默认 None / 5 m；None 表示按进口实际检测区上游端点生成到达线，数值表示显式上游距离；出口仍向下游 5 m。不是停止标线校准 |
| create_collector(..., detection_config=...) | 用户提供完整检测线配置，替换自动生成；含 map_name、junction_id、lines。字段示例见旧 configs/detection_lines.json，但坐标必须来自当前地图 |
| sample(snapshot, phase=None) | 采集并落盘，返回同一帧的字典；可选传入调度信息，重复/逆序 frame 报错 |
| packet | run_id、frame、sim_time、junction_id、vehicles、signals、phase、vehicle_context、frame_gap、read_overlapped_tick；类型见 CONTRACT.md |
| close(status) | 关闭文件和本采集器碰撞传感器，保存结束状态；可重复调用，不销毁外部车辆 |

坐标为 CARLA 世界坐标，位置 m，速度 m/s，加速度 m/s²，角度 degree，时间为仿真秒。
运行内车辆主键是 run_id+vehicle_id；车道主键是 road_id+section_id+lane_id，跨地图还需地图标识。
packet 是评估真值，不等同于 VLA 相机/雷达等模型输入，不能默认直接暴露给受测模型。
红绿灯智能体是否直接使用队列/占有率等评估真值，用户决定留待双方后续讨论；车辆智能体输入另行约定。
提供评估数据接口不代表默认授权任何策略使用这些数据，当前不自动连接策略观测。
车辆行缺少字段或 CSV 空值表示未知；signals 只有灯色；可选调度信息放在 packet.phase 和 phases.csv，
不从灯色推断剩余时间。新增首车、队列、空间占有率、路口数量与相位字段见 [指标说明](METRICS.md)。

## 职责边界

| 负责人/模块 | 负责 | 接合处 |
| --- | --- | --- |
| 采集与评估开发 | 地图读取、坐标匹配、原始真值、事件、指标和质量校验 | api.py、collector.py 与输出 schema |
| 车辆/信号智能体开发 | 模型输入、推理、动作映射、信号控制策略 | 从主程序拿当前 frame，提交下一帧动作；具体消息格式共同确定 |
| 主循环（双方约定唯一实现） | 世界设置、tick、动作执行顺序、超时策略、生命周期 | tick → sample → 决策 → 下次 tick 前应用动作 |
| 原固定配时/TM脚本 | 可复现实验基线 | 不作为智能体模块依赖，不和新主循环同时运行 |

需与同事确定的内容：同进程还是进程间通信、车辆动作类型、信号动作是选相位还是逐灯状态、
动作有效帧与超时回退、黄灯/全红约束归属、相位调度信息由谁提供。不在采集器内猜测这些规则。

## 当前能力边界

- 一次采集器对应一个路口。可在同一个主循环中对同一 snapshot 顺序调用多个采集器，
  但各自输出独立 run；全网去重、统一 run、共享车辆扫描/传感器和性能优化尚未完成。
- 无信号路口可记录车辆、拓扑进入/驶出及计数断面；现有等待/车道统计依赖唯一信号关联，
  对无信号进口不提供这些统计。不要把缺失统计当成没有停车。
- 分叉、短连接道、复杂路口和自动检测线冲突需要用户输入校准线；可显式传 lines=[] 仅采原始轨迹。
  任意地图是适配目标，尚未完成跨地图现场验收。自动断面需检查是否跨越相邻路口。
- 当前采集区域是水平圆，立交/重叠区域需进一步约束高度及路网归属；车道查询仍使用三维车辆坐标。
- 信号关联保留 unique/ambiguous/unmapped；坐标不能单独确定共享进口道上的转弯意图。
- 默认车辆原点决定主车道；已基于车身与采样车道条带交叠计算空间占有率，跨线车辆可影响多个车道。
  1 m 道路采样属于几何近似，弯道与变宽车道仍需物理验证。

## 命令入口

在 newProject 目录执行：

```powershell
python -m evaluation.run_collection --list-junctions
python -m evaluation.run_collection --junction-id 1427 --frames 200
python evaluation/topology.py --junction-id 1427 --output evaluation/configs/my_calibration.json
python -m unittest discover -s evaluation/tests -v
```

独立监听命令不 tick，无法承诺零漏帧。topology 命令会绘制调试线并生成 PNG，目标 JSON 必须不存在。
固定配时基线可用 `control_junction_phases.py --collect --junction-id 1427`，
车辆生成器默认按可配置比例混合四轮及以上车型，排除无轮数属性蓝图；`--vehicle-blueprint` 仍可限定车型。
两个入口均可用 `--detection-config` 指定校准线；显式旧 `--inventory` 不再隐式加载 Town05 检测线。

车型比例、2 m 初始净间距与准备期剔除见 [混合车型说明](TRAFFIC.md)。
生成清单提供释放帧，运行期指标应使用 `summarize --spawn-manifest` 重算，不能将整队摆放时的等待直接计入结果。
保留角色名 `junction_geometry_probe` 只用于临时读取车型包围盒，采集器明确跳过这些 actor。
