# SUMO 第一阶段联合训练

本目录是 CoTV 风格的第一阶段训练实现，和 CARLA 运行代码分开。设计对应
`/home/zlj/Drive/repos/CoTV/flow/examples/exp_configs/rl/multiagent/CoTV_grid.py`：

* `sim_step=1s`、`target_velocity=15m/s`、最大加/减速度 `3m/s²`；
* 灯控 actor 输出 4 个相位动作；车辆 actor 输出连续加速度 `[-3, 2]m/s²`；
* PPO 使用 `gamma=.999`、GAE `lambda=.97`、10 次 SGD epoch；
* critic 使用灯控观测和车辆观测的集中式拼接，执行时两个 actor 分别只看自己的观测。

没有 SUMO 配置时可以运行合成环境验证训练流程：

```bash
PYTHONPATH=. /home/zlj/lmdrive_runtime/env/bin/python \
  -m integrations.cross3.sumo_joint.train \
  --output outputs/cross3_sumo_joint/synthetic_seed20261102 \
  --episodes 20 --horizon 180 --cav-count 10
```

接入真实 SUMO 时提供 `.sumocfg`、信号灯 ID 和受控 CAV ID：

```bash
PYTHONPATH=. /home/zlj/lmdrive_runtime/env/bin/python \
  -m integrations.cross3.sumo_joint.train \
  --sumo-cfg /path/to/cross3.sumocfg --tls-id junction0 \
  --cav-id cav0 --cav-id cav1 \
  --output outputs/cross3_sumo_joint/sumo_seed20261102 \
  --episodes 2000 --horizon 720 --cav-count 10
```

`env.py` 负责 TraCI 状态采集、相位执行、CAV 加速度执行和安全裁剪；
`continuous_mappo.py` 负责连续高斯车辆策略、离散灯控策略、集中 critic、GAE
和 PPO clipped 更新；`train.py` 负责 rollout、checkpoint 和指标记录。第一阶段
模型迁移到 CARLA 时，车辆 actor 输出仍解释为目标加速度，交给 CARLA 安全层和
PID/MPC 执行，VLA 暂不参与训练。
