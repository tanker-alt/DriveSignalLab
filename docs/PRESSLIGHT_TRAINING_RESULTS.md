# PressLight 训练结果记录

## 一、离线预训练

训练脚本为 `integrations/cross3/offline_train.py`。状态采用官方 Cross3 PressLight 的 80 维布局：8 位相位灯态、12 条入口车道的 36 个位置分箱、12 条出口车道的 36 个位置分箱。动作共 4 个：南北直行、南北左转、东西直行、东西左转。

离线环境是随机队列仿真器，不是 CARLA 或 CityFlow。车辆到达率、初始排队、放行能力和下游离开过程均由随机模型生成。奖励采用官方默认的停止车辆奖励：每个时间步对停止超过 1 秒的入口车辆数乘以 -0.25。折扣为 `0.8 ** (实际决策时长 / 10)`。

100,000 步训练结果：

| 随机种子 | 固定配时成本 | MaxPressure 成本 | PressLight 成本 | PressLight 剩余车辆 |
|---:|---:|---:|---:|---:|
| 2026 | 10191.96 | 10330.75 | 9386.30 | 358.8 |
| 2027 | 10702.54 | 10833.99 | 9570.71 | 355.5 |
| 2028 | 10134.29 | 10229.19 | 9333.39 | 351.6 |

这些结果只反映离线随机队列环境，不能直接当作 CARLA 性能结论。

## 二、CARLA 微调配置

CARLA 版本为 0.9.12，RPC 端口为 31600，地图为 Cross3 OpenDRIVE。运行器使用与离线模型一致的 80 维状态和 4 个动作，并执行黄灯、全红、最小绿灯、最大绿灯和车辆清空安全逻辑。

CARLA 训练样本包含：当前 80 维状态、动作、实际决策持续时间、停止车辆奖励、下一状态和终止标记，同时保存 `signals.jsonl`、`vehicles.jsonl` 和 `summary.json`。

离线 seed 2028 权重已转换为：

```text
outputs/cross3_offline_official/offline_seed2028_carla80.pt
```

首次 30 秒烟雾微调完成 4/12 辆车，碰撞 0，信号错误 0；该结果只验证模型加载和运行链路，不用于性能结论。

## 三、正式 CARLA 微调结果

本节记录至少 180 秒完整回合的结果。固定配时、MaxPressure 和 PressLight 必须使用相同车流、相同随机种子和相同安全约束。

| 控制器 | 完成车辆/计划车辆 | 碰撞 | 信号错误 | 平均等待 | 排队成本 | 输出目录 |
|---|---:|---:|---:|---:|---:|---|
| 固定配时 | 240/240 | 0 | 0 | 52.95 秒 | -2734.33 | `/home/zlj/Drive/outputs/cross3/experiment_fixed_2lane` |
| MaxPressure | 240/240 | 0 | 0 | 40.96 秒 | -1309.18 | `/home/zlj/Drive/outputs/cross3/experiment_maxpressure_2lane` |
| PressLight 微调后 | 12/12 | 0 | 0 | 未计算 | 未计算 | `/home/zlj/Drive/outputs/cross3/offline80_carla_finetune_180s` |

正式结论必须以完成率、安全检查、平均等待和不同随机种子下的稳定性为依据，不能只比较训练损失或单回合奖励。


## 四、保守微调后的独立冻结评估

使用训练阶段未参与的种子 8401–8405、每条车道 2 辆背景车、每回合最多 180 秒，对离线模型和微调模型进行冻结评估。

| 模型 | 完成率 | 碰撞回合 | 平均完成时间 |
|---|---:|---:|---:|
| 微调前离线 seed 2028 | 100% (120/120) | 0/5 | 52.95 秒 |
| 20 回合 CARLA 微调后 | 80% (96/120) | 1/5 | 47.59 秒 |

微调模型在未碰撞回合中的完成时间更短，但整体安全性下降，不能接受为最终模型。当前训练结果说明“只按效率优化”会牺牲安全性；后续模型选择必须把零碰撞和 100% 完成率作为硬门槛，再比较完成时间。训练输出为 `/home/zlj/Drive/outputs/cross3/presslight_safe_training_fresh20`，冻结评估输出为 `/home/zlj/Drive/outputs/cross3/eval_presslight_before5` 和 `/home/zlj/Drive/outputs/cross3/eval_presslight_after5`。

## CoTV 风格灯车联合训练（2026-10-30）

参考 `/home/zlj/Drive/repos/CoTV` 的 `CoTV` 多智能体环境、PPO/GAE 配置和分离的 `tl`、`cav` 策略，本仓库新增了可迁移到 CARLA 的联合训练骨架：

- `integrations/cross3/vehicle_intent_policy.py`：车辆智能体输出四类离散意图（无额外引导、平缓接近、准备停车、条件允许时起步），只生成建议，不直接修改车辆控制或信号灯。
- `integrations/cross3/joint_mappo.py`：灯控 actor、车辆意图 actor、集中式 critic 和 PPO clipped/GAE 更新。灯控动作仍经过现有安全层。
- `integrations/cross3/train_joint_mappo.py`：在不启动 CARLA 的情况下验证联合 rollout 和更新流程；真实 CARLA 接入时由主循环唯一拥有 `world.tick()`，VLA 保持冻结 `eval()`。

每个决策周期发送给 VLA 的 `notice_text` 应使用真实信号与协商结果拼接的格式，例如：

`真实信号：NS_STRAIGHT，剩余约8.0秒，允许通行。协商结果：灯控智能体建议保持当前相位。车辆建议：若当前信号和前方空间均允许，建议平缓起步。`

本次离线 smoke 训练配置为 seed=20261030、3 回合、每回合 16 个 1 秒决策周期，成功完成联合 actor/critic 更新，产物写入 `/tmp/joint_test.pt`。正式 CARLA 训练建议先用 10--20 回合、每回合 180--300 秒、24--48 辆背景车和 2 辆 VLA 车做意图响应测试，再扩大规模；碰撞回合不保存模型，完成率必须达到 100%。

## SUMO 第一阶段连续加速度训练（2026-11-02）

新增 `integrations/cross3/sumo_joint/`，参考 CoTV 的 `CoTV_grid.py` 和 PPO 设置。灯控 actor 输出离散相位动作，车辆 actor 输出连续加速度 `[-3, 2] m/s²`；使用 `target_velocity=15 m/s`、`sim_step=1 s`、`gamma=.999`、GAE `lambda=.97`、10 次 PPO epoch。`env.py` 支持真实 TraCI，也提供无 SUMO 时的合成接口回归模式。合成 3 回合 smoke 训练已通过，产物仅用于代码验证；正式结果必须使用真实 SUMO 随机交通流评估后再迁移 CARLA。

本次补充了 CoTV 1x1 四岔路口参数清单 `outputs/cross3_sumo_joint/cotv_grid_1x1.json`：四个进口道各 1 条车道，边长 300 m，70 辆车，流量为右 120、下 240、左 288、上 192 veh/h，horizon=720 秒，完全对应 `CoTV_grid.py`。当前服务器未安装 `sumo` 和 Python `traci`，因此暂时只能完成配置生成和合成回归，尚未声称真实 SUMO 训练结果。

## SUMO 安装与真实训练状态（2026-11-02）

已在 `/home/zlj/lmdrive_runtime/env` 安装 `traci==1.18.0`、`sumolib==1.18.0` 和 `eclipse-sumo==1.18.0` 的 Python 组件，并生成真实四岔路口文件：

```text
scenarios/cotv_grid_1x1/cotv.net.xml
scenarios/cotv_grid_1x1/cotv.rou.xml
scenarios/cotv_grid_1x1/cotv.sumocfg
```

但受当前服务器网络策略限制，TraCI 进程监听 8813 端口返回 `Operation not permitted`；`libsumo` 二进制包下载也因代理超时未完成。因此本次没有伪造真实 SUMO 训练结果，真实训练命令和配置已经准备好，待端口权限或 `libsumo` 安装可用后直接运行。

## 2000 回合结果与训练策略调整（2026-11-03）

真实 SUMO 2000 回合已完成且碰撞回合为 0，但 reward 从前 100--200 回合的约 -71 至 -86 退化到最后 100 回合约 -105，说明后期训练出现策略退化。loss/value loss 下降只能说明优化数值稳定，不能替代交通效果指标。训练器现已改为默认 500 回合，支持每 10 回合独立 checkpoint、滑动平均奖励最佳模型和 early stopping；新增 `sumo_joint/evaluate.py` 用相同随机种子比较 fixed、MaxPressure 和 learned。

## CoTV 原版链路验证（2026-11-04）

已建立 Micromamba Python 3.7.3 环境并安装 Ray 0.8.0、TensorFlow 1.15.2、Gym 0.14.0、Flow 原版依赖。通过 `integrations/cross3/cotv_original/run.py` 调用上游 CoTV 原始 `train_ppo.py`；1 次 smoke 迭代成功，随后 20 次原版 PPO 迭代成功，共 16000 环境步，episode reward 从约 -2308 降到 -757。运行目录为 `outputs/cotv_original/train20`。为适配当前机器仅降低 workers/rollouts 到 1 并启用 Ray local mode，环境、tl/cav 策略、动作、奖励和 PPO 更新均来自 CoTV 原版；SUMO 版本为 1.18，不是论文使用的 1.10，因此这是原版链路验证而非精确论文复现。
