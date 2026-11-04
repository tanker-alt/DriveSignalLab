# CoTV 原版复现入口

本入口调用 Guojyjy/CoTV 原始 Flow 环境和 RLlib PPO，不使用相邻 `sumo_joint/` 的自写算法。

## 来源

- 源码：`/home/zlj/Drive/repos/CoTV`
- URL：https://github.com/Guojyjy/CoTV
- 已核查提交：`5ce1a95cc52d7a108294488a701025b488787f07`
- 配置：`flow/examples/exp_configs/rl/multiagent/CoTV_grid.py`
- 环境：`flow/flow/envs/multiagent/traffic_light_grid.py` 中的 `CoTV`
- 入口：`flow/examples/train_ppo.py`

`run.py` 先检查依赖并保存 manifest（源文件 SHA256、提交、运行参数）。通过检查后把源码复制到独立输出目录，再调用原始入口，不修改原始观测、奖励、动作和 PPO 更新。

## 已核实的原版设置

| 项目 | 设置 |
|---|---|
| 路网 | Flow 生成的 1×1 路网，单车道，路段参数 300 m |
| 灯控动作 | Discrete(2)：保持 / 请求切换；原版黄灯逻辑 |
| 车辆动作 | 连续加速度 [-3, 3] m/s² |
| CAV 观测 | 9 维 |
| 灯控观测 | 此配置为 27 维 |
| 受控车辆 | 按进口道动态选择 CAV，并非固定 cav0 的动作广播给全部车辆 |
| 灯控奖励 | 负的进口、出口占用差 |
| 车辆奖励 | 原版 min_delay_edge - 1 - stable_acceleration_positive_edge |
| PPO | 两类共享策略 tl/cav；非自写 MAPPO |
| 网络 | [32, 32, 32] |
| gamma / lambda | 0.999 / 0.97 |
| SGD 次数 | 10 |
| rollout | 720 步，每步 1 秒 |
| 批次 | 18 个 rollout，共 12960 环境步 |
| workers / GPU | 18 / 1，保持原版要求，尚未验证资源可用性 |

`total_veh=70` 在原版中用于构造车辆类型抽样池，不能当成任意实验的实际完成车辆数。

## 运行语义

```bash
python3 integrations/cross3/cotv_original/run.py --check \
  --python /path/to/isolated-python37 --sumo-home /path/to/sumo-1.10.0
```

去掉 `--check` 后默认只执行 **1 次训练迭代** 的运行验证。`--iterations` 对应原版 `--num_steps`，不是 episode 数。初步训练可考虑 18–28 次迭代（名义上 324–504 个 rollout），但必须先验证原版采样与评估。不要直接运行 500 次迭代。

输出包括 `manifest.json`、`train.log`、原版源码运行副本及 Ray 输出。错误不会降级为合成交通环境。

## 2026-11-04 验证状态

- 原版文件及哈希核查完成，预检查按预期报告缺少独立 Python 3.7 和 SUMO 1.10.0。
- 原版 environment.yml 指定 Python 3.7.3、Ray 0.8.0、TF 1.15.2、Gym 0.14.0。
- 当前 CARLA 虚拟环境为 Python 3.8，已有 SUMO 1.18.0，不满足原版环境要求；未更改它。
- Docker 查询与下载 Python 3.7 安装器的提权请求均被自动审批后端故障阻断：404 / gpt-5.6-luna 不受支持。没有执行这些受限操作。
- **尚未完成原版依赖安装、SUMO rollout 或训练。** 脚本只完成预检查，不能据此宣称复现成功。
- 原 `sumo_joint` 的历史结果存在实现错误，不作为 CoTV 复现或优于 MaxPressure 的证据。

## 原版 smoke/小规模训练结果（2026-11-04）

已在独立 Micromamba 环境 `/home/zlj/micromamba/root/envs/cotv` 跑通原版入口。为适应当前主机，将 CoTV 原配置中的 18 workers/18 rollouts 降为 1 worker/1 rollout，并使用 Ray local mode；CoTV 环境、tl/cav 策略、奖励和 PPO 代码保持原版。SUMO 使用已安装的 1.18.0，并在私有运行副本中做了 TraCI `setColor` 关键字兼容。

1 次迭代成功：720 秒 episode、800 环境步、reward=-2112.77。随后 20 次迭代成功完成，共 16000 环境步，最终 `episode_reward_mean=-757.04`，训练日志为 `outputs/cotv_original/train20/train.log`，Ray 状态为 `outputs/cotv_original/train20/ray_results/CoTV_1x1grid/`。日志中的 reward 从约 -2308（第 1 次）下降到 -757（第 20 次），说明原版 PPO 在该小规模设置下确实在学习。

这不是论文精确复现：主机使用 SUMO 1.18 而非 CoTV 论文的 1.10，且并行度降低；结果只用于验证原版代码链路和训练趋势。
