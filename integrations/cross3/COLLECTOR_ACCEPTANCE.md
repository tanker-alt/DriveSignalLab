# Cross3 交付采集器接入验证（2026-10-24）

## 结论与范围

交付包原有 `evaluation.api.create_collector` / `FrameCollector.sample` 能在本机
CARLA 0.9.12 上采集 Cross3 的车辆与信号真值，并将同帧字典返回给外部调用者。
55 个交付清单文件 SHA256 均未改变。测试适配器位于同级 `probe_collector.py`，
没有用自写采集器替代上游采集器。

动态验证报告：`Drive/outputs/cross3/collector_probe_context_final_20261024/report.json`。
12 辆测试车，192 个连续采样帧，12 种逐灯放行状态及 4 种多灯相位组合。
10161 项检查通过；0 丢帧、0 跨 tick 读取；最高速度 5.743 m/s。
逐帧核对车辆数、信号数、车辆对应 signal ID、灯色、快照位置及速度；检测区内 12 辆车的几何上下文均唯一匹配。
前两轮分别覆盖静态状态及精细检测区外动态状态。补测期间原 31600/31610 服务均已退出；
重启独立 31610 实例、加载 Cross3 后完成本轮验证，测试车辆均已清理。
测试车辆由探针直接控制，属于接口试验，不是遵守红灯的交通闭环验收。

`agent_input` 只接收采集结果，不直接访问 CARLA。已输出每帧的
`agent_observations.jsonl` 和可阅读的 `agent_observation_example.json`，包括
12 条车道的灯态、车辆数、静止车辆数、车辆明细和车辆上下文。
这验证了外部程序能够消费接口；未将这些真值输入 LMDrive 模型，也未验证模型会利用它决策。

## 接入方式

在现有唯一 tick 主循环内创建一次采集器，每次 tick 后调用一次 sample：

```python
from evaluation.api import RoadSpec, create_collector
from evaluation.map_profile import read_profile, adapt_world, validate_world

profile = read_profile(upstream / 'map_profiles/generated_cross3/profile.json')
collection_world = adapt_world(world, profile)
validate_world(collection_world, profile)
collector = create_collector(collection_world, RoadSpec(junction_id=10),
                             output_root=output_root)
try:
    # 下列两行放入既有主循环；不要额外启动第二个 tick 进程。
    world.tick()
    observation = collector.sample(world.get_snapshot())
    # 外部智能体适配器读取 observation，组织自己需要的输入。
finally:
    collector.close()
```

运行时按 OpenDRIVE signal ID 绑定，不能复用加载前的 actor ID。
collector 不控制灯色，不生成车辆，也不推进世界。
`sample` 返回 vehicles、signals、vehicle_context、frame、sim_time 等；
同时写出 CSV。部分聚合指标目前仅输出到 CSV，不等于已提供统一在线策略输入。

## 尚需区分的事项

1. 原始 `phases.csv` 的 `active_signal_opendrive_id` 是单值字段。交付说明明确指出
   新四相位控制器尚未接入原采集器的多灯相位语义。本探针不伪填这个单值字段，
   在外部适配器中单独传递 phase 和 active_signal_ids；原始实际灯态仍完整采集。
2. 默认基础采集半径 120 m，精细进口几何检测长度 50 m。区外 geometry_match=unmapped
   不等于车辆或信号完全丢失。调用者必须处理缺失与歧义，不能当成零距离。
3. reference_only 是参考断面，不能把 front_reference_distance_m 当作已标定停止线距离。
4. 本机 Cross3 训练入口此前用的是 run_cross3.py 的 observation()，不是上游 FrameCollector。
   不能用此前训练成功替代本次上游接口验证。
5. 采集器队列速度阈值为 1.0 m/s，本机训练入口为 0.3 m/s；检测范围也不同。
   更换 PressLight 状态来源前须统一车道顺序、检测范围、阈值和状态维度。
6. LMDrive 的 RGB/LiDAR 等传感器输入独立存在。将此采集器真值作为额外路侧信息，
   需要明确增加消息/模型输入通道，不能声称当前已经接入或等同传感器观测。

本次不验收通行量、完整排队/占有率指标、碰撞记录器、闯红灯评分或模型收益。
