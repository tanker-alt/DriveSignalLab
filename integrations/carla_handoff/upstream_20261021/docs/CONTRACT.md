# 当前可执行数据契约

本文件说明现有代码，不将设计草案当作已实现 API。入口为 evaluation.api；
工作流见 SETUP.md，指标定义见 METRICS.md，协作设计见 INTEGRATION.md。
发生冲突时，当前接口以本文件及代码为准；标注为历史的工作日志描述当时行为。

## 版本与边界

| 对象 | 当前版本/状态 |
| --- | --- |
| Python 公共 API | 文档版本 0.1；RoadSpec / create_collector / sample / close |
| 原始实验 metadata.schema_version | 1.2；phases 新增一致性列属于向后兼容附加字段 |
| 派生 summary.schema_version | 1.0；指标字典可能追加字段 |
| 地图工具导出 | schema_version=map_tools.1；不等同于 detection_config |
| 智能体消息/动作适配器 | 草案，尚未提供可执行 API |

读取 CSV 应按列名而非列号；允许未知附加列。旧数据缺列应视为 unavailable，不能补零。
CSV 缺失值为空字符串，Python/JSON 缺失值通常为 None/null；数值 0 是有效观测。
geometry_match=unique 仅表示唯一几何归属，metric_status=valid 不保证队列尾部全部可见。

## 调用入口

RoadSpec(junction_id, radius_m=120.0, expected_map=None, arrival_distance_m=None, exit_distance_m=5.0)。
create_collector(world, roads, output_root=..., mode="orchestrator_owned_tick", collisions=False,
detection_config=None, stop_reference_config=None)。地图由调用方加载，采集器不 tick、不执行智能体动作。
arrival_distance_m=None 使新实验按实际进口检测区上游边界生成到达线；显式数字是距离覆盖。
缺少无信号进口几何时必须提供显式检测配置/距离，完整等待等指标仍不支持无信号进口。
detection_config 是整套替换，stop_reference_config 是同断面停止参考 s 覆盖；详见 METRICS.md。

sample(snapshot, phase=None) 每帧调用一次，重复或倒序帧报错；返回以下普通 Python 字典：

| 字段 | 类型与含义 |
| --- | --- |
| run_id | str，当前采集器实验 ID |
| frame / sim_time | int / float，CARLA 帧号/仿真秒 |
| junction_id | int，当前地图路口 ID |
| vehicles | list[dict]，与 vehicles.csv 同次采集的车辆记录 |
| signals | dict[str,str]，OpenDRIVE 信号 ID 到实际状态字符串 |
| phase | dict，传入的调度字段；未提供时仅 source=unavailable |
| vehicle_context | list[dict]，几何匹配、逐车参考距离及入口信号关联 |
| frame_gap | int，当前与上一采集帧之间缺失帧数 |
| read_overlapped_tick | bool，采集期间是否跨越其他 tick |

当前包不返回 approaches/lanes/flows/junction 汇总；这些仅写文件，实时汇总 API 属于待讨论改进。
signals 来自 actor 缓存；严格同帧依赖唯一 tick 和信号执行者。车辆运动学来自 ActorSnapshot；
控制量 throttle/brake/steer 来自 actor 缓存，control_source=actor_cache_between_ticks。
close(status="completed") 保存最终状态并释放本采集器资源，不销毁外部车辆；可重复调用。
主循环或人工中止分别应传 failed/interrupted，不能将异常默认标成 completed。

## 单位、标识与信号语义

- 世界位置、几何距离为米，速度 m/s，加速度 m/s²，yaw/pitch/roll 为度，时间为仿真秒。
- vehicles.speed_mps 是三维速度模；队列/首车/进口平均速度使用水平速度模，二者不能无条件混用。
- longitudinal_accel_mps2 在原始车辆表中按车身 yaw 投影；首车字段按道路切向投影。
- 运行内车辆键为 run_id+vehicle_id；车道键为 road_id+section_id+lane_id；跨地图必须附地图指纹。
- signal IDs 为 OpenDRIVE 字符串；actor ID 是当前世界实例 ID，不跨重载稳定。
- 当前 reference 距离不等于真实路面标线；retained_entry_signal 是历史入口关联，不保证实际转向控制关系。
- 队列按当前坐标/速度/车身间距判断，不以到达事件为前提；完整区间等待则要求观测覆盖与配对。
- 红绿灯和车辆智能体能否直接使用真值留待双方讨论，接口数据不自动进入模型输入。

## 输出索引

| 文件 | 内容 |
| --- | --- |
| metadata.json、frames.csv | 实验身份、设置、范围、状态和逐帧完整性 |
| vehicles.csv、signals.csv | 原始车辆与实际灯色 |
| phases.csv | 调度/倒计时；控制器提供 signal_consistency 与 signal_mismatch_ids，其他调用方未提供时为空 |
| vehicle_context.csv | 沿进口路径的逐车距离与信号来源 |
| approaches.csv、lanes.csv、junctions.csv、flows.csv | 队列/首车/占用、进口等待均值、路口汇总和断面流量 |
| events.jsonl、trips.jsonl、summary.json | 事件、通行配对及最终统计 |
| collisions.jsonl | 可选原始碰撞与传感器覆盖记录，不等于事故计数 |
| detection_lines_used.json、approach_geometry_used.json | 复算使用的检测线/几何快照（几何不可用时可能缺失） |
| controller_config.json、signal_checks.jsonl | 固定控制器独有：配置及逐帧期望/实际灯色比对 |

原始 CSV 用 UTF-8 BOM，JSON/JSONL 用 UTF-8。运行中逐帧刷新文件缓冲，不等于磁盘 fsync。
回放生成新的 analysis 子目录，不改原始文件，也不重建历史缺失的控制量或相位。
