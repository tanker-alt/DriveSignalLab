# 分阶段验收协议

当前第一阶段已通过，第二阶段按冻结协议执行；第三阶段只有在第二阶段取得独立改善证据后才开放。
原 [VALIDATION.md](VALIDATION.md) 是 v1 先导记录，不是新协议的通过凭据。

## 第一阶段：测量可信

新协议为 `cross3_transfer_v2` / `cross3_metrics_v2`，不混用 v1 权重或指标。

修正并验证了以下事项：

- SUMO 与 CARLA 都按车头通过出口边界计数；计划到达 → 正确出口的时间包括生成队列。
- 两域停止线均为进口道路 s=144 m。SUMO 导入网络显式增加 6 m stopOffset，
  初始车头位置按车长换算，使初始车身中心对应 s=60 m。
- 实际生成等待与仿真内停车等待分别统计；保留总等待。物理队列与外部队列不再混称。
- 按实际时间差积分，拒绝重复/缺失时间段、非法速度、未知消失车辆、重复需求 ID。
- 原始每帧位置、速度、车体尺寸、车道、测得灯色和事件写入压缩日志；独立脚本从
  生命周期解析式与原始速度区间复算，不调用线上 Metrics。
- 增加车头红灯越线检测。黄灯进入单独记录，不冒充红灯违规；绿灯进入后变红不重复计数。
- 增加恒速、固定航向、有向矩形的交叉冲突 TTC（最多预测 10 s）。这是风险代理指标，
  不是未来真实轨迹或碰撞保证。手算案例与独立静态多边形重叠验证计算结果。
- 同车道追尾间距使用沿车道距离，并报告有效观测时间；无接近时不是 TTC=0。

实际发现并修复：旧背景 BasicAgent 没识别专用右转信号，固定配时虽 15/15 完成却
有 3 次红灯越线。仅在背景驾驶器中用车道—信号绑定和车头停止距离修复；不覆盖 VLA 控制。
原始失败证据保留在 `outputs/staged/stage1_carla_probe/`。

正式验收目录：`outputs/staged/stage1_acceptance_native/`。
23 项测试、10 个真实场景、11,871 帧，112 辆完成、26 辆在刻意截断场景中保留未完成。
补充验证了 SUMO 毫秒精度向上取整，保证实际发车不早于计划到达。
所有独立复算通过，正式基线观测到的红灯越线与碰撞均为 0。完整场景覆盖两个种子、
固定配时/MaxPressure、SUMO/CARLA；截断场景验证“只统计成功车辆”的偏差不会发生。

部分逐车核对（同一计划需求，车辆 v8，西进口右转）：

| 测量 | SUMO | CARLA |
|---|---:|---:|
| 计划到达 / s | 3.458456 | 3.458456 |
| 实际生成 / s | 3.500000 | 4.350000 |
| 车头过停止线 / s | 47.004000 | 44.126384 |
| 越线时物理灯色 | GREEN | GREEN |
| 正确出口完成 / s | 63.900000 | 60.450001 |
| 外部等待 / s | 0.041544 | 0.891544 |
| 仿真内等待 / s | 27.200000 | 18.650000 |

逐车完整 CSV、原始日志、复算结果均附 SHA256；此核对不要求两域数值相等，因为
背景驾驶动力学和发车空间约束不同，而是确认各自统计能从物理记录独立还原。
瞬时外部队列使用模拟器当帧可见车辆列表；时间积分使用实际 departure 时间，
两者在 SUMO 发车离散化边界可能有一个 tick 的可见性差异。

## 第二阶段：先证明 SUMO 策略有效

训练与验证协议在运行前写入 `protocol.json`，默认：

- SUMO 训练 100 回合，种子 20001 起；四类需求轮换：均衡、东西偏重、南北偏重、左转偏重。
- 独立验证种子 23001 起，8 场；固定配时与 MaxPressure 使用相同需求。
- 只在验证集选择第 25/50/75/100 回合的候选模型；首先看碰撞/红灯违规和未完成数，再看等待。
- SUMO 必须全部完成、无观测到的碰撞/红灯违规、平均等待不差于固定配时。
  MaxPressure 作为更强对照如实报告，不隐瞒差距。
- 通过后仅评一次独立测试集（种子 24001 起）；不通过时不得继续针对这批测试种子调参。
- 只有 SUMO 验证及测试通过，才生成 `sumo_acceptance.json` 并进入 CARLA。

CARLA 使用独立微调种子 25001 起，验证 26001 起，最终测试 27001 起。
微调前、固定配时、MaxPressure、微调后具有匹配需求。正式第三阶段门槛是：
全部完成、无观测到的碰撞/红灯违规，且微调后平均等待相对直接迁移至少降低 5%，
并在独立测试中再次成立。该 5% 是预先指定的工程筛查门槛，不是统计显著性证明。
当前只使用一个训练随机种子；论文级稳健性仍需多个训练种子与置信区间。

不满足门槛时保存失败报告，分析相位服务、方向饥饿和状态/执行差异；不自动进入第三阶段，
也不把“参数更新了”或训练奖励上升当作验收通过。

### 第二阶段已执行的学习诊断

普通 PPO 已按 100 回合、四种需求训练，并对第 25/50/75/100 回合进行冻结验证。
最佳候选仍有 53/221 辆未完成，平均等待 59.10 s；固定配时与 MaxPressure 均
221/221 完成，平均等待分别为 31.07 s 和 20.70 s。该方案没有通过 SUMO 门槛，
没有启动 CARLA 微调，也没有查看独立测试集。失败证据在 `outputs/staged/stage2_native/`。

后续方案显式使用 **MaxPressure 先验 + 可训练神经残差 logits 的 PPO**：
`logits = 3 × 每相位压力(辆) + 神经网络残差 + 很小的同分保持偏置`。
初始化残差为零；采样、记录概率、PPO 更新使用同一分布，最终动作仍服从安全 mask，
没有在采样后偷偷改成规则动作。残差可以改变动作选择，检查点保存先验类型，避免
相同权重被不同策略语义解释。这个先验是方法的一部分，不把它描述为从零学出的能力。

同样的 100 回合预算与验证集复测，并额外评估未训练的先验策略。验证集允许方法选择，
选择完成后才开放独立测试集。运行可在 stage2 命令加 `--signal-prior maxpressure`；不加时
继续使用普通 PPO。当前正式结果以 `outputs/staged/stage2_pressure_prior/` 的报告为准。

### 压力先验方案的实测进展与审计修复

`stage2_pressure_prior` 中验证选出第 50 回合检查点。验证平均等待 20.56 s，
未训练先验为 20.70 s，学习增益约 0.7%，主要优势来自显式压力先验。
独立 SUMO 测试 229/229 完成，固定配时等待 31.14 s，模型 20.83 s，
无观测到的碰撞与红灯越线。

CARLA 直接迁移 8 场为 225/225 完成，平均等待 27.50 s，无观测到的碰撞与红灯越线。
首次独立审计发现最大外部队列 10 辆被复算为 9 辆：复算把下一步 `before` 中
才出现的车辆追溯为上一步已经生成。已改为按帧因果累积可见车辆，并增加手算回归案例。
修正后 8 场均通过审计。模拟器、在线指标、策略和奖励均未改变。

由于审计代码改变，旧凭据按协议失效，必须重跑第一阶段和第二阶段才能继续。
此前的 SUMO 通过及 CARLA 数据作为历史实测保留，不能借旧凭据开放第三阶段。
目前尚无 CARLA 微调改善的验收结果。

## 第三阶段：直接联合微调 VLA 内部层与灯控

保留已经实现的内部轨迹层 PPO，输入来自真实 LMDrive 传感器推理，更新其内部
40,970 个输出层参数与灯控 actor；不是外部意图 actor，也不是整个 7B 骨干全参数训练。
执行前必须提供前两阶段验收凭据及对应模型，程序核对权重 SHA256 和继承链。

第三阶段先做相同需求下“冻结 VLA + 已适配灯控”对照，再联合微调，在独立场景验证
效率是否改善且安全/驾驶质量不退化。前一轮的短程联合运行仅算链路测试，不算本阶段通过。

## 执行

```bash
cd /home/zlj/DriveSignalLab
export PATH=/home/zlj/lmdrive_runtime/env/bin:$PATH
export PYTHONPATH=/home/zlj/DriveSignalLab/outputs/staged/runtime:.
export LIBSUMO_AS_TRACI=quiet
PY=/home/zlj/lmdrive_runtime/env/bin/python
# 若尚未安装，可隔离安装匹配当前 SUMO 1.18.0 的库：
# $PY -m pip install --target outputs/staged/runtime --no-deps libsumo==1.18.0

# 需要一个空闲的 CARLA 31600 实例；入口拒绝接管已有活动世界。
$PY -m integrations.cross3.transfer.stage1 \
  --output outputs/staged/metric_acceptance_NEW --port 31600

$PY -m integrations.cross3.transfer.stage2 \
  --metrics-acceptance outputs/staged/metric_acceptance_NEW/acceptance.json \
  --output outputs/staged/transfer_study_NEW --port 31600

# 只有第二阶段通过并生成两个凭据，以下第三阶段命令才会被允许。
$PY -m integrations.cross3.transfer.run --backend carla --port 31600 --train --vla \
  --metrics-acceptance outputs/staged/metric_acceptance_NEW/acceptance.json \
  --sumo-acceptance outputs/staged/transfer_study_NEW/sumo_acceptance.json \
  --carla-acceptance outputs/staged/transfer_study_NEW/carla_acceptance.json \
  --checkpoint outputs/staged/transfer_study_NEW/carla_train/checkpoint_019.pt \
  --waypoint-head /home/zlj/Drive/models/lmdrive/cross3_waypoint_head.pt \
  --episodes 10 --seed 28001 --output outputs/staged/joint_NEW
```

所有输出目录必须全新。指标/仿真代码变化会令旧验收失效；证据被修改、丢失或模型不在
已验收继承链中也会被拒绝。这是自动证据检查，不是再次请求用户批准。

进程内 libsumo 与 TCP 接口的同场景 1,083 帧原始测量逐字节一致，见
[transport_parity.json](transport_parity.json)。启用 libsumo 只改变调用方式，不替换真实 SUMO。
