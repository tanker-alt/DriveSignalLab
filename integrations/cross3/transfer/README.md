# SUMO → CARLA → VLA，逐阶段验收

从 [分阶段协议与运行说明](STAGES.md) 开始。当前采用 v2 指标与模型契约，
旧 [先导记录](VALIDATION.md) 只保留历史证据，不能作为阶段验收通过。

1. `stage1.py`：手算测试 + 真实 SUMO/CARLA + 独立逐车复算，生成指标验收凭据。
2. `stage2.py`：SUMO 训练/验证/独立测试 → CARLA 迁移/微调/独立测试。
3. `run.py --vla --train`：前两阶段通过后，实际联合更新 LMDrive 内部轨迹输出层与灯控模型。

关键模块：

- `core.py`：时间、等待、删失、流量和共享状态定义。
- `safety.py`：物理停止线越线、追尾 TTC 和交叉冲突 TTC 风险代理。
- `measurements.py` / `audit_metrics.py`：原始逐帧证据与独立复算。
- `stages.py`：阶段凭据、代码哈希、证据哈希和检查点继承校验。
- `sumo_env.py` / `carla_env.py`：同一 Cross3 地图、明确车道映射、唯一仿真时钟。
- `learning.py`：带原始动作掩码、截断 bootstrap 的 PPO；联合时用保存的真实 VLA
  隐特征对内部最后一层反向传播，更新后同步回 VLA，不反向传播经过模拟器。

每次运行保留源码快照、需求、逐车指标、物理灯态、压缩测量日志和独立 checkpoint。
训练额外保留 `rollout.pt`；大权重/图像/日志留在 `outputs/`，Git 记录代码与小型验收报告。

指标不是完整道路安全认证。交叉 TTC 使用恒速/固定航向假设，未完成车辆的观测时间是
完整旅行时间的下界。比较时必须同时看完成率、等待、碰撞、红灯越线和各方向服务情况。
