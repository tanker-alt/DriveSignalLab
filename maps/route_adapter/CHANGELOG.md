# 2026-11-10：地图拓扑与边界 OD 工具包

基于 `maps` 分支 `22db2002598103692cfa1de3e84f1d632b93186f` 更新原
`maps/route_adapter`，保留已有地图、连续路线和四个规划器。

- 保留 `load_route`、`carla_locations`、`spawn_transform` 与旧 CLI。
- 新增 `adapt`、`build_od_catalog`、`audit_route`、车道方向/换道/绕行辅助接口及 `od` CLI。
- 固化 `map_adapter.2` JSON Schema，增加明确的错误码、信号/停止线哈希与映射诊断。
- 由 OpenDRIVE 车道段构建图，避免 CARLA topology 列表遗漏短连接器后的终端段；端点探测不再跳过 5 cm 连接器。
- BFS 在入队时去重，以父节点表重建路径；闭合图、不可达出口和不足边界净距的道路明确失败。
- 增加逐地图进程隔离兼容性工具，记录原生异常退出、超时和完整来源哈希。

## 验证范围

Windows / Python 3.7 / CARLA Python API 0.9.15，无 CARLA Server。
发布副本的 16 项测试覆盖旧接口、输出 schema、X/T/R OD、反向/错误出口/
缺失连接器/越界路线、损坏信号与停止线、短连接器、闭合图，以及报告异常分支。

|已发布地图|OD 数量|
|---|---:|
|cross3|12|
|t_north|6|
|roundabout4_8groups|32|
|cross_1x6|42|
|cross_3x3|36|
|t_1x6|16|
|t_3x3|18|
|heterogeneous_1x6|30|
|heterogeneous_3x3|34|
|heterogeneous_3x3_roundabout|52|

上述 10 张地图共 278 个 OD。另验证本地 4 张派生图（共 14 张/314 个 OD），
以及使用当前生成器新生成的 X/T/R（12/6/32 个 OD）。派生图、场景规格、
本地验收报告和运行输出不包含在此次提交中。可用 README 中的矩阵命令复现。

## 边界

边界 OD 每对只保留一条 successor 路径，不主动换道，不保证米制最短路径。
入口/出口保留 15 m 净距；单段同时作为起终点时需要超过 30 m。
小于等于 0.2 mm 的车道段明确拒绝。闭合路网需未来显式起终点接口。
特殊路权、信号策略、车身碰撞、死锁、交通安全和动态驾驶不在离线认证范围内。
CARLA 原生解析风险通过矩阵子进程隔离，XML 引用检查不是完整 XSD 验证。
