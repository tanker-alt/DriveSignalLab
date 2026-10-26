# 交付文件结构与阅读顺序

交付 ZIP 解压后的根目录即项目根目录。先读 README.md，再读 SETUP.md、CONTRACT.md、INTEGRATION.md。
所有命令从根目录运行。以下为交付结构，省略部分内部模块：

```text
项目根目录/
├── README.md                      总入口和运行命令
├── requirements.txt               核心 CARLA 客户端依赖
├── requirements-visualization.txt  可选 PNG 绘图依赖
├── DELIVERY_MANIFEST.json          交付文件路径、大小及 SHA256
├── control_junction_phases.py      固定配时基线、唯一 tick、实际灯色核验
├── spawn_junction_traffic.py       混合车队生成、TM 托管和清理
├── traffic_scenario.py             车型分配、实际包围盒及初始净距计算
├── map_tools/                     原 codex，更名后的地图调查工具包
│   ├── __main__.py                python -m map_tools：查地图/路口/道路，可选标签与视角
│   └── README.md                  工具用法及旧脚本处理说明
├── evaluation/
│   ├── api.py                     对外入口：RoadSpec、create_collector
│   ├── collector.py               同帧真值采集与返回包
│   ├── recorder.py                派生指标计算调度及输出
│   ├── map_context.py、topology.py 地图拓扑与计数断面生成
│   ├── approach_geometry.py       进口路径几何快照
│   ├── approach_metrics.py        首车、连续队列、空间占有率
│   ├── vehicle_tracker.py         已观测停车等待、进入/驶出与截断
│   ├── trip_tracker.py            到达—出口配对和区间等待质量
│   ├── signal_association.py      坐标唯一信号及历史入口关联
│   ├── detection_lines.py         有方向断面过线检测
│   ├── metrics.py                 车道均值与滚动流量
│   ├── collision_recorder.py      原始碰撞与传感器覆盖记录
│   ├── run_collection.py          独立监听入口，不负责 tick
│   ├── summarize.py               离线回放，不覆盖原实验
│   ├── configs/                   默认参数与地图示例（地图示例不是跨地图通用配置）
│   ├── tests/                     无需服务器的回归测试，仍需 CARLA Python 包
│   └── outputs/                   交付时仅保留历史 Markdown；运行后生成新实验
├── tools/
│   ├── selfcheck.py               环境/可选服务器只读自检
│   └── build_delivery.py          白名单导出新目录和 ZIP
├── docs/
│   ├── SETUP.md                   安装、版本、自检和最短步骤
│   ├── CONTRACT.md                当前可执行接口、类型、单位和输出文件
│   ├── INTEGRATION.md             分工、时序与尚未实现的消息草案
│   ├── METRICS.md                 指标定义、校准和验收限制
│   ├── METRIC_STATUS.md           原始 14 项指标追踪（包含历史验证节点）
│   ├── TRAFFIC.md                 混合车型和净间距
│   ├── DECISIONS.md               已确认决策及讨论记录
│   ├── PENDING_DISCUSSIONS.md     待双方讨论/后续实现事项
│   ├── HANDOFF_REVIEW.md          交付前审查记录
│   ├── PHASE_FIX_2026-10-21.md     相位根因与修复证据
│   └── PROJECT_STRUCTURE.md       本文件
└── WORKLOG_*.md                   工作日志；10-20 文件追加了 10-21 总结和 10-22 待办
```

接入方主要修改自己的模型适配器和主循环，先通过 evaluation/api.py 使用采集器。
固定相位脚本与 Traffic Manager 生成器属于实验基线，不需要把它们嵌进模型。
现有包尚未包含可运行的智能体适配器；文档草案不代表实现完成。

仅发送方本地保留：map_tools/legacy 旧脚本/日志、旧地图 JSON、根目录生成清单、
实验 CSV/JSONL、大量历史原始数据、map_exports 和 dist 验证目录。它们不进入正式代码包。
全部 Markdown 均保留；历史报告指向未随包发送的原始文件属于正常情况，不是运行依赖缺失。
新 ZIP 使用新版本名称，旧 ZIP 是历史交付快照，不能混用其旧 codex 命令。
