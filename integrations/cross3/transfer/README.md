# SUMO → CARLA 迁移与 VLA 内部联合微调

实测结果与已知问题见 [验收报告](VALIDATION.md)。

此入口使用真实 SUMO/CARLA，不会自动退回合成环境。它与旧的
`sumo_joint/`、`official/`、CoTV 原版权重及车辆残差控制器隔离。

## 训练的是什么

1. SUMO：四动作信号 actor + critic。
2. CARLA 灯控微调：加载同一份 SUMO actor/critic/优化器，使用真实 CARLA 转移继续 PPO。
3. `--vla --train`：同时更新上述灯控 actor 和 **LMDrive 内部
   `waypoints_predictor.2` 的权重及偏置**。不是外部意图 actor，也不是教师模仿学习。

联合阶段采用参数高效微调：视觉编码器、语言模型、轨迹头前层冻结；内部最后一层
从真实传感器产生的隐特征预测 5 个二维路点，在训练采样时加入标准差 0.05 m 的
高斯探索，再由原 PID 执行。保存该层的输入、实际采样路点、行为均值与 log probability。
集中 critic 结合交通状态和两辆车状态；全体车辆的交通回报同时用于灯控 PPO 和
轨迹头 PPO。每个一秒区间内所有轨迹采样的 log probability 相加，构成该区间车辆
策略的联合概率比。一次回合内行为参数保持不变，回合结束才更新。

使用 CPU FP32 master 参数更新内部最后一层，再同步回实际 VLA 模块；不需要保留
7B 模型计算图。评估使用均值轨迹，不加入探索。适配头和信号权重保存在同一 checkpoint。
上游 `no_grad()` 只影响采样；更新时最后一层确实反向传播。日志报告两类参数变化量。
这是内部输出层联合微调，**不是整个 VLA 全参数训练**。算法可运行不等于性能已提升。

## 统一协议

- 同一 Cross3 OpenDRIVE 导出 SUMO 路网，逐项验证进口、出口和 12 个信号 link。
- 动作固定为 NS 直行 / NS 左转 / EW 直行 / EW 左转，右转随同向直行。
- 状态 20 维：四组排队 / 进口 / 出口计数，当前相位 one-hot，绿黄红状态，阶段年龄。
  所有车道有明确语义，不使用 Python `hash()`。
- 上层 1 s；SUMO 0.1 s、CARLA 0.05 s。绿灯至少 5 s、黄灯 3 s、全红至少 2 s；
  冲突区域尚有车辆时延长全红。有其他方向需求且绿灯达 20 s 时，动作 mask 禁止继续保持。
- PPO 重新计算概率时使用采样时的原始 mask。环境不把强制执行动作冒充采样动作。
- 连续生成到达需求，超时保留价值 bootstrap；全体车辆完成/碰撞为真终止。
- 奖励：全体已到达需求的车辆时间负积分 / 20，碰撞额外 -100。排队等待生成的车辆也计入。
- checkpoint 严格校验协议与地图。旧 13 维哈希车道 SUMO 模型无法可靠转换，明确拒绝加载；
  原版 CoTV 二相位权重也不能冒充 Cross3 四相位模型。

CARLA 和 SUMO 保持相同需求种子、路线、发车位置、目标出口与限速，但车辆动力学、
感知、驾驶器以及停车线几何存在差异。相同地图不代表两域动力学相同。

## 指标口径

每车从**计划到达时间**开始计时，CARLA 到正确出口 `s<=88`，SUMO 到对应出口
`arrivalPos=62` 结束。未生成的排队需求不会消失；尚未到达的未来需求不算未完成。

- 完成率、未完成数和未来需求数必须同时报告。
- 完成车辆的平均/P95 旅行时间明确标注 completed；不能单独用于选模。
- `censored_vehicle_time_s` 是所有到达需求截至结束的累计车辆时间；未完成车辆是右删失，
  `observed_time_per_due_vehicle_s` 只是观测下界，不能称为完整平均旅行时间。
- 等待：速度 <0.1 m/s 的累计时间，包含外部生成队列。
- 停车：低速累计至少 1 s 才记一次，速度 >0.5 m/s 后重新允许计次，避免静止抖动反复计数。
- 吞吐率：实际完成数 / 实际测量时长。提前清空与满时长场景要结合需求和完成率解释。
- TTC：仅同车道追尾，使用净车间距 / 正接近速度。SUMO getLeader 的 minGap 被加回；
  非接近、无前车返回无有效 TTC。报告最小 TTC 和低于 1.5 s 的车辆秒。
- 碰撞是回调/模拟器检测证据；当前不包含独立闯红灯和交叉冲突 TTC 检测，这两项为 `null`，
  **不能解释为零违规或安全认证**。
- 按进口记录等待时间，暴露方向饥饿；不把流量不均衡下的原始等待总和直接称为公平性得分。

`compare.py` 拒绝训练回合、不同需求、不同时间窗和不同 VLA 配置的比较，按相同种子
输出配对差值。至少多种子、独立验证与测试后才能判断是否改善；不能用训练奖励代替评估。

## 本机运行

```bash
cd /home/zlj/DriveSignalLab
export PATH=/home/zlj/lmdrive_runtime/env/bin:$PATH
export PYTHONPATH=.
PY=/home/zlj/lmdrive_runtime/env/bin/python

# 先训练新协议下的 SUMO 模型；输出目录必须不存在。
$PY -m integrations.cross3.transfer.run --backend sumo --train \
  --episodes 100 --arrival-seconds 120 --duration 420 \
  --seed 2026 --output outputs/transfer/sumo_train

# 使用空闲 CARLA 实例。不要与其他实验共用世界。
$PY -m integrations.cross3.transfer.run --backend carla --port 31600 \
  --checkpoint outputs/transfer/sumo_train/checkpoint_099.pt \
  --episodes 5 --seed 8401 --output outputs/transfer/before

# 独立训练种子微调，源 checkpoint 不会被覆盖。
$PY -m integrations.cross3.transfer.run --backend carla --port 31600 --train \
  --checkpoint outputs/transfer/sumo_train/checkpoint_099.pt \
  --episodes 20 --seed 9001 --output outputs/transfer/finetune

# 微调后匹配评估。
$PY -m integrations.cross3.transfer.run --backend carla --port 31600 \
  --checkpoint outputs/transfer/finetune/checkpoint_019.pt \
  --episodes 5 --seed 8401 --output outputs/transfer/after

# 相同条件的基线；可把 fixed 换成 maxpressure。
$PY -m integrations.cross3.transfer.run --backend carla --port 31600 \
  --controller fixed --episodes 5 --seed 8401 --output outputs/transfer/fixed

$PY -m integrations.cross3.transfer.compare outputs/transfer/fixed \
  outputs/transfer/before outputs/transfer/after --output outputs/transfer/comparison.json

# 两辆真实 LMDrive + 背景交通，直接联合微调内部轨迹层与灯控。
$PY -m integrations.cross3.transfer.run --backend carla --port 31600 --train --vla \
  --checkpoint outputs/transfer/finetune/checkpoint_019.pt \
  --waypoint-head /home/zlj/Drive/models/lmdrive/cross3_waypoint_head.pt \
  --episodes 10 --seed 10001 --output outputs/transfer/joint

# 联合后评估；保持相同初始适配头，checkpoint 中的联合训练层会覆盖它的最后一层。
$PY -m integrations.cross3.transfer.run --backend carla --port 31600 --vla \
  --checkpoint outputs/transfer/joint/checkpoint_009.pt \
  --waypoint-head /home/zlj/Drive/models/lmdrive/cross3_waypoint_head.pt \
  --episodes 5 --seed 9401 --output outputs/transfer/joint_test

$PY -m unittest integrations.cross3.transfer.test_transfer -v
```

`--waypoint-head` 为显式场景适配，不是原始模型泛化结果。联合前后的 VLA 评估均要带
`--vla`，使用相同初始适配头和需求；不要拿纯规则车辆基线与 VLA 运行直接归因比较。

每轮输出协议、源码快照及哈希、需求、决策、逐车指标、参数变化和独立 checkpoint。
CARLA 另存位置诊断、实际内部路点采样以及原 LMDrive 输入/输出审计。模型权重和大日志
留在 `outputs/`，不提交 Git；Git 中保留源代码、测试及小型验收报告。
