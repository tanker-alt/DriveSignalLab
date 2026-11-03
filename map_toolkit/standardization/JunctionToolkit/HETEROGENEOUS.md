# 异构道路随机路网

现有两车道三叉、三车道四叉模板通过 scenariogeneration LaneDef 和 create_lane_links_from_ids 拼接，可选加入一个双车道环岛。

## 配置

```json
{"name":"heterogeneous_3x3_roundabout","rows":3,"columns":3,"spacing_m":500,"seed":42,"include_roundabout":true}
```

include_roundabout 默认 false。开启时恰好一个环岛，默认位置 [rows // 2, columns // 2]，可用 roundabout_position: [行, 列] 指定（从零计数，偶数尺寸选中间偏大索引）。其余格子三叉/四叉各 0.5，三叉缺失方向各 0.25。

保留原有整轮筛选：普通格子至少两个时要求两类普通路口都有，要求连通且存在覆盖所有逻辑格子的路线。最终分布受筛选影响。保存的 config.json/layout 可复现；固定 layout 必须与指定环岛位置一致且至多一个环岛。无环岛最小间距 445m，有环岛 450m。

## 人行道

连接道路两侧增加 0.2m 路缘和 2m 人行道，高度 0.15m，与模板一致。随最外侧驾驶车道渐变，跨 laneSection 和两端建立双向链接。嵌入环岛的四条外接道路、四段环行弧线及八条外侧入口/出口弯道均补齐同规格人行道，另保存 16 条路缘/人行道连接。环内直行连接不加路缘，避免横挡车辆出入口；不包含中央岛步行设施或行人导航。

外围人行道修正版：`standardization/maps/heterogeneous_3x3_roundabout_sidewalks_20261103`。沿用原布局，CARLA 再次实车行驶 4407m、覆盖 12/12 实际路口及完整环行，零碰撞、零重生、零传送。`carla_trial_20261103/` 保存报告、轨迹及 `ring_sidewalk_overview.png` 实景俯视图。

默认 simple 和 off 均沿用全路网原生人行道材质，不给环岛单独增加混凝土贴面。原生人行道及其连接仍完整保留。仅 complete 模式调用 `ring_sidewalk_finish.py`，添加环岛外围及接入道路末端 12m 的无碰撞混凝土外观；完整模式仍有局部贴面范围边界。历史俯视截图包含这层贴面，不代表当前默认 simple 的显示效果；不宣称行人通行验收。

渐变段前后各 20m 等宽，中段第三车道以三次多项式增减宽度。两条内侧车道贯通，第三车道收窄仍需主动并线，不伪造多对一跳转。环岛保留四个实际 OpenDRIVE 入口路口及内部连接，manifest 另外记录所属逻辑格子。

## 生成

工作区根目录运行，输出目录必须为空：

```powershell
& ./generator-env/Scripts/python.exe -B standardization/JunctionToolkit/toolkit.py heterogeneous --config standardization/JunctionToolkit/configs/heterogeneous_3x3_roundabout.json --out standardization/maps/heterogeneous_ring_new
```

heterogeneous 命令现在仅生成并执行 XML 静态检查，不导入、不启动、不连接 CARLA。检查 ID/引用、接缝双向链接、人行道连续性、环岛内部连接及覆盖逻辑格子的抽象路线。static_validation.json 绑定 XODR 哈希，明确记录 carla_validation_performed=false、driving_acceptance=not_run。

静态通过不代表车辆实际行驶通过。现已适配嵌入环岛的 CARLA 路线验收：进入环岛后完整绕行一圈，再驶出并继续穿越路网，同时核对实际路口顺序、环行道路顺序和连接段跨越证据。

## CARLA 验收（2026-11-03）

两张下列示例均已在 CARLA 0.9.15 中实车通过，地图 XODR 未改动，报告与其 SHA256 绑定。无环岛版行驶约 4190m，覆盖 9/9 实际路口；中心环岛版约 4407m，覆盖 12/12 实际路口，包括环岛四个入口和完整一圈。两次均通过 8 处路线连接段，零碰撞、零重生、零传送，最大跟踪误差均约 0.69m。

每张地图的 carla_acceptance_20261103.json 汇总拓扑、路线、轨迹及跟车截图路径。此次是全绿灯、单物理车辆的连通性验收，不代表红绿灯控制策略或多车合流验收。首次 Low 画质加载发生引擎地形渲染任务崩溃，记录在无环岛版 carla_trial_20261103/load_failure.json；Epic 重启后完成两张地图验收，尚不能确定单一崩溃原因。

复验可分别执行（需要空闲 CARLA 服务；所有视角默认跟车）：

```powershell
& ./carla-env/Scripts/python.exe -B standardization/JunctionToolkit/validate_heterogeneous.py standardization/maps/heterogeneous_3x3_roundabout_20261103
& ./carla-env/Scripts/python.exe -B standardization/JunctionToolkit/network_route_check.py standardization/maps/heterogeneous_3x3_roundabout_20261103 --port 2002 --report-subdir replay_new --realtime --keep-vehicle
```

--realtime 按仿真时间播放，省略时加速执行但仍更新跟车视角。验收入口拒绝覆盖已有车辆或同步会话。本次环岛版车辆 298 保留在端口 2002 的终点。

本次示例：

- standardization/maps/heterogeneous_3x3_sidewalks_20261103：3 个三叉、6 个四叉，11 段连接道路。
- standardization/maps/heterogeneous_3x3_roundabout_20261103：4 个三叉、4 个四叉、中心 1 个环岛，10 段连接道路；9 个逻辑格子对应 12 个实际 OpenDRIVE 路口。

旧 heterogeneous_3x3 及历史 CARLA 报告保留，历史结果不适用于新地图。本次未复制到主工程。
