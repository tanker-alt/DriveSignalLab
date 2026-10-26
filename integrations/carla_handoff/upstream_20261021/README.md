# CARLA 车灯协同评估平台：工作入口

交付版阅读顺序：[安装、自检、最短运行](docs/SETUP.md) → [地图/路口工具](map_tools/README.md)
→ [当前数据契约](docs/CONTRACT.md) → [开发协作](docs/INTEGRATION.md)。
运行产物在 evaluation/outputs；精简交付目录和 ZIP 由 python -B -m tools.build_delivery 生成。
历史报告保留用于追溯，原始实验数据不随精简包发送。
完整文件结构和模块职责见 [PROJECT_STRUCTURE.md](docs/PROJECT_STRUCTURE.md)。

当前基线：Town05 / 路口 1427，混合车型、Traffic Manager 随机转向。
采集入口现已改为读取用户当前加载的地图，用户提供目标路口与可选校准断面。
跨地图代码已完成首轮离线验证，真实地图现场验收仍待进行。
目标：支持用户接入 VLA 或信号智能体，输出单车驾驶指标和路口运行指标；
按单路口单 VLA → 多车/多 VLA → 多路口 → 异构路口扩展。
当前多辆 TM 车辆仅用于验证基础设施，并不表示 VLA 接入已完成。

## 新成员阅读顺序（2026-10-21）

打包前的接口缺口、依赖与发布范围见 [协作交付检查](docs/HANDOFF_REVIEW.md)，建议按顺序讨论并落实。

1. [协作与接口契约](docs/INTEGRATION.md)：职责、时序、输入输出与限制。
2. [精简代码入口](evaluation/api.py)：RoadSpec / create_collector / sample / close。
3. [分轮讨论问题](docs/DECISIONS.md)：车型、几何、队列、占有率和相位接口。
4. [今日整理和验证记录](WORKLOG_2026-10-21.md)。
5. [混合车型和净间距](docs/TRAFFIC.md)：比例、车型白名单、2 m 净距、准备期剔除。
6. [新增指标说明](docs/METRICS.md)：双首车、队列、空间占有率、路口内数量、逐帧相位和控制量。

10 月 21 日真实测试已核查：24 车、4,323 帧无缺帧，净距通过；
[检查与回算报告](evaluation/outputs/20261021T101348Z_f0470857/inspection_report_20261021.md)。
新采集沿用下方命令，自动增加 approaches.csv、junctions.csv、phases.csv 和几何快照。

## 正式入口

相位不一致已修复：控制器在接管前设置 TM 种子（--tm-seed，默认 17）；生成器 --seed
只控制本地车队随机，运行中不再重置 TM。沿用下面命令即可。逐帧灯色校验失败时会保存证据并停止实验。
原因、在线验证和新字段见 [相位修复报告](docs/PHASE_FIX_2026-10-21.md)。

```powershell
# 终端一：唯一 tick 所有者；保持运行到车辆清理结束
python control_junction_phases.py --collect --collisions
# 终端二：24 车，运行 180 仿真秒后自动清理
python spawn_junction_traffic.py --vehicles-per-approach 6 --route random --duration-seconds 180
```

车型默认 random，可用 --vehicle-blueprint 限定。车辆退出核验后，再 Ctrl+C 停控制器。
random 现在按 `evaluation/configs/traffic.json` 分配类别，初始权重 60/20/15/5（轿车/SUV/货车/公交），
净间距默认 2 m，可用 `--spawn-gap` 覆盖。整队摆放期间的停车要通过
`python -m evaluation.summarize <run目录> --spawn-manifest <本次清单路径>` 从运行期指标中剔除。
--reset-all-signals 仅用于修复旧全图冻结，正常运行不要附加。

## 文件职责与保留规则

日常协作只需从 `evaluation/api.py` 和 `docs/INTEGRATION.md` 开始；运行使用根目录两个脚本。
最新指标缺口见 [METRIC_STATUS.md](docs/METRIC_STATUS.md)，逐项讨论顺序见 [DECISIONS.md](docs/DECISIONS.md)。
2026-10-21 已清理 31 个 Python 字节码缓存、3 个空缓存目录及旧权限探针；所有 Markdown 保留。
缓存会在后续 Python 运行时自动重建。旧调查脚本/输出归档在 map_tools/legacy，生成清单与历史实验仍保留用于复算。

| 位置 | 职责 |
| --- | --- |
| control_junction_phases.py | 固定配时基线、同步推进、采集生命周期 |
| spawn_junction_traffic.py | 可配置测试车、TM 指令、生成清单、批量清理 |
| traffic_scenario.py | 车型数量分配、车道几何投影、实际包围盒与初始净距 |
| evaluation/api.py | 面向协作开发的简短公共接口 |
| evaluation/map_context.py | 读取已加载地图、发现路口、生成实时信号拓扑 |
| evaluation/approach_geometry.py、approach_metrics.py | 可回放的进口道几何与首车/队列/空间占有率 |
| evaluation/topology.py | 从地图生成并绘制进口/出口流量检测线 |
| evaluation/collector.py | 同帧车辆真值与信号采集 |
| evaluation/vehicle_tracker.py | 等待、路口进入/驶出与观测截断 |
| evaluation/detection_lines.py | 有方向有限线段过线检测 |
| evaluation/metrics.py | 车道停车/速度/等待和滚动流量 |
| evaluation/collision_recorder.py | 碰撞原始回调、安装与关闭覆盖记录 |
| evaluation/recorder.py | 派生结果持续落盘 |
| evaluation/run_collection.py | 独立监听调试入口，不保证逐帧完整 |
| evaluation/summarize.py | 旧数据回放，分析保存至独立子目录 |
| evaluation/configs/ | 指标阈值、当前检测线、原停止参考线备份与俯视图 |
| evaluation/tests/ | 离线回归测试 |
| evaluation/outputs/ | 每轮原始数据、派生分析、检查报告；不覆盖、不挪动 |
| map_tools/ | 当前地图发现、路口/道路查询与可选标签；旧脚本在 legacy 本地归档，不进入交付代码 |
| spawn_manifest_*.json | 每次测试车辆的车型、种子、请求动作和实际生成信息 |
| junction_1427_*.csv | 旧低频调试日志，可能覆盖；评估以 outputs 为准 |

原 codex 已更名为 map_tools，旧脚本/日志在 map_tools/legacy；旧地图 JSON 随目录迁移，实验 outputs 不挪动。
原规划中的 communication.py（车灯消息层）尚未实现；不创建空文件冒充完成。

详细字段、限制和操作说明见 [evaluation/README.md](evaluation/README.md)。
历史总结见 [WORKLOG_2026-10-20.md](WORKLOG_2026-10-20.md)，最新决定以今日记录为准。
