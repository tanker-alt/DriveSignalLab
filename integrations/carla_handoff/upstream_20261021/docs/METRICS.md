# 新增进口道指标与逐帧相位（2026-10-21）

## 队列受控验收：保留初值（2026-10-21 用户确认）

最新优先级：先验收检测区内初始化车队。队列算法直接读取每帧车辆坐标/包围盒/速度，
不依赖 arrival 事件，也不要求完整通行记录；车辆在区内出生不妨碍当前队列识别。
到达流量与完整等待是独立指标，覆盖不足时继续输出未知/截断。

对 110350Z 实验增加独立直线投影核验：选取中心线偏离首尾直线小于 1 mm 的 6 条进口，
从原始世界坐标和包围盒顶点计算原点归属、车头/车尾纵距及车间净距，不调用生产投影函数。
运行期 33528 条进口帧记录中，主车道数量、队列数、队首/队尾 ID 均无差异；
首车距离与队列长度最大差异约 0.00000705 m。这是相同地图参考下的独立几何算法对照，
不证明真实停止标线准确，也不替代弯道/跨线的受控物理验收。详情见该实验检查报告。

继续使用队列速度 ≤1 m/s、相邻净距 ≤5 m、队首参考距离 ≤10 m；允许缓行车辆属于队列。
停车等待仍使用独立 0.1/0.3 m/s 滞回，不把“属于队列”直接当作“正在停车等待”。

本轮离线对照已执行：3 辆不同长度（4/6/10 m）的已知矩形车辆，前后区间分别为
[2,6]、[9,15]、[18,28] m，速度 0.5/0.8/1 m/s。手算队列 3 辆、长度 26 m，
50 m 检测区纵向占有率为 20/50=0.4（净间隙不计占用）。另核验三个阈值的等于/略超、
中间移动车辆截断队列、孤立尾车、长车跨线主归属与分别占用、车身超出检测区的占用截断。
全套 54 项测试通过。这些是算法对照，不是 CARLA 实车物理真值。

在线场景清单（尚未执行，CARLA localhost:2000 本轮连接超时）：

| 场景 | 设置与独立预期 |
| --- | --- |
| 两车停车 | 平直进口，首车车头距参考点约 2 m、净距约 3 m；两车应成队，队列长度包含间隙，占用长度不含间隙 |
| 三车不同长度 | 用实际不同车型，三车净距均约 3 m；按实测包围盒计算预期车头、车尾和占用，不假设统一车长 |
| 尾车断开 | 前两车连接，第二与第三车净距约 6 m；队列仅前两车，但第三车仍贡献区域占用 |
| 缓行与移动中车 | 连续观测速度约 0.8 m/s 应可成队；中车实测超过 1 m/s 后，后车不跨过它连接到前队 |
| 长车跨线 | 原点留在本车道，车身跨入相邻车道；主车道人数只计一次，两个车道分别有占用 |
| 检测区边界 | 车尾跨过上游边界时占用按区域裁剪；区域外车队不能宣称已完整观测 |

先在直线、近似等宽车道验证，再验证弯道和变宽车道。实际速度以快照与位置差分核对，
不能把控制指令的目标速度当真值。直线车身参考距离/净距用世界坐标与独立直线投影手算，
不要用 ApproachMetrics 的投影函数生成验收期望。临界值离线精确核验；在线先选远离阈值的场景，
避免把落地漂移、速度波动与几何近似误差误认为边界逻辑错误。
在线实验须记录车型、实际位置/速度、几何快照、配置和观测帧，再根据误差决定是否调参。

## 新实验到达边界更新

用户已确认新实验到达线对齐各进口实际检测区的上游边界。自动生成使用已保存进口几何的
最后一个采样点，法向指向下游；目标 50 m 遇截短时按实际 42/47 m 等长度生成。
检测线快照记录 arrival_policy、approach_id、actual_detection_length_m 与截短状态。
用户 detection_config 整套覆盖优先；RoadSpec.arrival_distance_m 数值可显式指定距离。
缺少进口几何或共享上游车道有歧义时要求显式配置，不退回隐含的 10 m。
拓扑工具默认同步采用此策略，--arrival-distance-m 可覆盖。
旧实验与 summarize 使用原检测线快照，不自动迁移；新旧区间等待均值不可直接混作同口径。
到达线移动扩大区间，不自动保证等待观测完整；信号不明、预热和截断仍按原质量规则排除。
停止参考点覆盖会重建进口区域，因此也会影响自动对齐的到达线；显式配置的检测线不随之移动。

## 逐车距离、信号来源与双等待口径

最新状态见 [METRIC_STATUS.md](METRIC_STATUS.md)。新增 vehicle_context.csv（每帧每车）：
geometry_match 为 unique/ambiguous/unmapped/unavailable；唯一进口匹配时输出 approach_id、
origin_reference_distance_m、front_reference_distance_m、rear_reference_distance_m。
距离沿采样道路路径、单位米，上游为正；仅限已建进口区域，缺少包围盒时车头/车尾距离为空。
reference_kind 默认为 carla_stop_waypoint；calibration_id 默认空。
approach_signal_id 优先坐标唯一匹配；进入目标路口后连续观测可保留入口信号，
signal_association_source=retained_entry_signal；缺帧、离开或丢失即清除。
这不是转向路线的控制关系证明。approach_signal_state 只取同帧 signals，缺失为空。

junctions.csv 的 mean_current_approach_wait_s 是当前唯一信号匹配且尚未驶入/通过的车辆，
本次观测累计进口等待的车辆加权均值；同时提供人数和平均速度。
其人群沿用 lanes.csv，不等于 approaches.csv 的 50 米几何人群。空人群均值为空。

trips.jsonl 将 actor 原点正向通过到达线与出口线配对；同车重复访问分别记录。
要求中间连续观测且曾进入目标路口。travel_time_s 为两次插值过线时刻之差。
approach_wait_s 是两次过线之间的进口停车等待，沿用停车滞回阈值，不是全程延误。
边界等待累计值在相邻帧间线性插值；精度受采样周期和等待阈值影响。
mean_paired_approach_wait_s 只以有效配对等待为分母，wait_sample_count 显示分母，
wait_excluded_paired_count 显示配对但等待覆盖不足数量。
缺帧、丢失、重复到达、无到达的出口、实验结束等单列 status/reason；
预热、信号不明、等待基线/终点缺失等列入 wait_issues，不作为零等待样本。
逐帧 open_trip_count 在结束前显示未完成访问；关闭时写入截断记录，summary.json 为最终统计。

## 用户停止参考校准接口

命令行 run_collection.py、control_junction_phases.py、evaluation.approach_geometry
支持 --stop-reference-config <JSON路径>；create_collector 接收同内容字典 stop_reference_config。

```json
{
  "map_name": "Town05",
  "junction_id": 1427,
  "opendrive_sha256": "替换为当前地图 inventory 中的完整指纹",
  "approaches": {
    "道路ID_断面ID_车道ID": {"s": 18.0, "calibration_id": "人工测量记录ID"}
  }
}
```

键使用原始停止参考点的 road_section_lane，s 是该道路 OpenDRIVE 纵向坐标（米），
不是沿进口路径的偏移量。当前仅支持同 road/lane/section 内移动参考点。
地图名、路口和指纹必须匹配；无法解析、跨 section、未知进口和无校准 ID 拒绝执行。
用户须提供实际测量值，示例不是可直接使用的真实标线配置。
覆盖后 reference_kind=user_supplied_stop_waypoint，并保存校准配置及几何快照；
标签仅说明用户提供，不自动证明真实标线准确。
覆盖会重建进口几何，影响该区域首车、队列、占有率及逐车距离。
不改写原始 stop_plane_front_distance；新实验自动到达线随进口几何边界生成，出口线和显式检测线不随之移动。


沿用原启动命令即可启用。新增配置 `evaluation/configs/approach_metrics.json`：
上游目标长度 50 m、道路采样 1 m、排队速度 1 m/s、连接净距 5 m、队首距停止参考点 10 m。
这些为可调试验初值，尚未完成实地人工标注验收。原等待滞回 0.1/0.3 m/s 独立保留。

## approaches.csv：每帧每个已建几何进口区域一行

| 字段 | 定义 |
| --- | --- |
| approach_id | 停止参考点的 road_section_lane，跨 road/section 的上游路径归入同一进口区域 |
| primary_vehicle_count | actor 原点匹配该进口路径、位于实际检测长度内的车辆数；共享路径无法唯一判断时未知 |
| lane_head_* | 主车道车辆按车头沿道路的参考距离排序，最靠前的车辆 ID、水平速度、道路切向加速度、距离 |
| queue_head_* | 从车道首车开始满足低速、距线条件并向后连续连接的队列首车；没有队列则为空 |
| queue_vehicle_count / queue_tail_id | 连续队列车辆数与最后一辆车 ID；遇运动中的车辆或超过净距的间隔即停止连接 |
| queue_length_m | 已观测队列从首车车头至末车车尾的纵向跨度，含车间隙；边缘车身可延伸出检测区，不等于停止线至队尾距离 |
| spatial_occupancy | 车辆凸包与采样车道条带交叠后，沿道路覆盖区间的并集长度 / 实际检测长度；跨线车可占用相邻车道 |
| detection_length_m | 实际可追踪的上游检测长度，不穿过分叉和相邻路口 |
| geometry_status | complete / upstream_end_or_branch / degenerate_geometry |
| metric_status | valid / missing_vehicle_geometry / ambiguous_primary_lane / no_lane_geometry；未知不按零处理 |

主车道车辆数与车身占用分开：同一辆跨线车只归入一个主车道，但占用可影响两条车道。
占有率采用纵向覆盖定义，极窄的跨线交叠也会贡献该交叠部分的纵向覆盖，不是车道面积占有率。
空车道输出零队列/零占有率，首车字段为空。缺少包围盒时不补用统一车长。
检测区在地图边界前截短时，统计仅代表这个范围，不能把观测队尾当作完整队尾。

所有距离以 CARLA 停止参考 waypoint 为起点，使用采样道路距离与车身投影，
仍需校准真实道路标线。纵向加速度是世界 ax/ay 在道路行进切向的投影，不是三维模长。
高度容差用于减少上下层道路误计；复杂立交、合流或宽度突变仍需专门验证。
当前几何来自信号停止参考点，无信号路口没有这些进口指标，仍可采集车辆和路口内数量。

## junctions.csv：区分车辆存量与过线次数

- inside_vehicle_count：本采集范围内，车辆原点所在 current_junction_id 等于目标路口的数量。
- exit_crossing_count：启用出口断面的累计正向过线次数，同车再次通过再次计数。
- exit_window_count：相同出口断面的最近 60 仿真秒过线次数。
- observed_vehicle_count：本帧整个采集范围内车辆数。

没有配置出口断面时两项出口计数为空；不是零。数量受采集半径覆盖限制，
当前路口内车辆不代表车身与路口相交的车辆，也不代表车尾完全清空。

## phases.csv 与 vehicles.csv schema 1.2

控制器逐帧提供 phase_id、stage、cycle、active_signal_opendrive_id、next_phase_id、
remaining_seconds、scheduled_start_time/end_time，source=fixed_schedule。
剩余时间由仿真时间和本段调度结束时刻计算，不读取被手动切灯干扰的自动计时器。
阶段边界最多存在一个 tick 的离散化误差；有限周期最后的下一相位为空。

独立采集或智能体没有提供调度时 source=unavailable，其余字段为空；不根据灯色猜相位。
智能体集成可调用 `collector.sample(snapshot, phase=phase_dict)`，传入同样字段并标记来源。
车辆新增 throttle/brake/steer/reverse/hand_brake，control_source=actor_cache_between_ticks。
控制量来自 actor 缓存，不在 ActorSnapshot 中；严格同帧依赖唯一 tick 与动作执行者。
longitudinal_accel_mps2 / lateral_accel_mps2 为水平加速度在车身 yaw 前向/侧向的投影。

## 回放与初始化剔除

新运行自动保存 approach_geometry_used.json，回放默认使用该快照，不需要连接服务器。
旧运行需要在相同 OpenDRIVE 地图仍可读时导出几何，并显式传给回放：

```powershell
python -m evaluation.approach_geometry --run <原实验目录> --output <新的几何JSON路径>
python -m evaluation.summarize <原实验目录> --spawn-manifest <对应生成清单> --approach-geometry <几何JSON路径>
```

检查地图名称、路口 ID、已记录的 OpenDRIVE 指纹，避免将不同地图的几何套入旧轨迹。
几何文件生成与回放不推进仿真。原始实验不改写，所有分析进入新的 analysis 子目录。
使用初始化车队时仍需 --spawn-manifest 排除准备阶段；新增空间指标不能消除准备期偏差。
历史数据没有记录的相位、控制量不补造。
