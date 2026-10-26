# Cross3 新地图接入（2026-10-24）

## 更换灯控或智驾模型

联合入口支持 `--signal-agent JSON` 和 `--driving-agent JSON`，可以分别替换灯控与智驾模型。
接口契约、适配器示例和官方环境评估命令见 [统一接入说明](../../agent_interfaces/README.md)。
现有 `live.sh` 仍沿用原来的固定模型组合；新插件通过 `run_cross3.py` 入口运行。

## VS Code 实时预览（一条启动命令）

确保 CARLA 31600 已启动且没有其他实验占用，然后运行：

```bash
bash /home/zlj/zlj/DriveSignalLab/integrations/cross3/live.sh
```

在 VS Code 的「端口」面板转发 **8768**，用该行「在浏览器中打开」。
实时页面沿用完整流程可视化布局，保留时间轴、相机、轨迹、曲线及历史训练对照。本地端口未被改写时，地址为 `http://127.0.0.1:8768/complete_process.html`。
此命令启动两辆适配后的真实 LMDrive、12 辆背景车、冻结 PressLight 策略和实时网页。
页面每秒更新双车相机、车辆位置示意、实际灯态、排队及完成进度；不是离线回放。
相机和轨迹独立采样，存在延迟；模型初始化时相机需要等待。
每次使用新的 `Drive/outputs/cross3/live_时间戳` 目录，旁边 `.log` 为运行日志。
运行完成后网页保留末帧，Ctrl+C 关闭预览并中断该命令启动的实验。
本入口不执行训练、不修改模型检查点。训练继续使用下方独立端口入口。
可用 `--port 其他端口` 改网页端口，`--carla-port 31610` 选择其他空闲 CARLA 实例。

完整运行可视化：`Drive/outputs/cross3/complete_process.html`，自包含、可离线打开。
本地预览地址：`http://127.0.0.1:8768/complete_process.html`。
包括同步的双主车真实摄像头、俯视轨迹、实际 12 灯状态、主车控制量与速度、排队变化、
训练回合和固定配时对照。图像按每个 ego 的 step 与审计 camera_frame 对齐，每约 0.5 秒抽样；
退休后保留最后一帧并明确标记完成。页面的俯视道路为示意图，车辆轨迹来自实测。

重建：`python3 DriveSignalLab/integrations/cross3/build_full_visualization.py`。

原交付隔离保存在 `../carla_handoff/upstream_20261023_v4/`，55 个清单文件
SHA256 全部核对通过，没有修改上游代码。压缩包来源哈希见
`../carla_handoff/SOURCE_20261023_v4.json`。旧 Town02/Town05 入口保持原状。

## 本机兼容性

本机服务端和 Python API 均为 CARLA 0.9.12；上游验收版本为 0.9.15。
真实服务端已核对 Cross3 OpenDRIVE 指纹、12 盏原生灯及全部 affected lanes。
0.9.12 的 `static.prop.mesh` 没有 `mesh_path`，因此使用原生 OpenDRIVE 外观，
没有恢复上游的铺装网格和官方装饰杆架；每轮 `map_load.json` 明确记录这一限制。
需要复原交付包完整外观时，应另行安装并验收 0.9.15，不能将当前画面描述为外观一致。

启动程序前须保证目标服务端没有活动车辆/传感器、没有同步实验。
每个进程只连接一个端口，独占该世界的 tick 和信号控制。
`run_cross3.py` 直接复用 MDrive 的 LMDrive 和 AgentWrapper，绕过默认
`load_world(TownXX)` 流程，避免自定义地图被旧入口换回 Town05。
它是独立验证运行器，不产生官方 Leaderboard 分数。

本机已验证 Vulkan 适配器编号 2 对应物理 GPU 1；它不等于 nvidia-smi 编号。
本次工作结束时，31600 与 31610 服务端均保留运行并加载 Cross3，实验车辆/传感器已清理，
世界恢复异步。服务仍在时跳过下面的服务端启动命令，直接运行实验入口。

```bash
cd /home/zlj/zlj
Drive/software/CARLA_0.9.12_MDrive/CarlaUE4.sh \
  --world-port=31600 -quality-level=Epic -graphicsadapter=2 -RenderOffScreen
```

## 四相位和背景交通

相位顺序固定为 NS_STRAIGHT、NS_LEFT、EW_STRAIGHT、EW_LEFT。
默认四个右转灯常绿，语义沿用交付包；清空阶段只有直左车道全红。
信号按 OpenDRIVE signal ID 绑定，禁止写死 actor ID。

固定配时每相位 20 秒绿、3 秒黄、2 秒直左清空，初始红灯 2 秒。
自适应控制最短绿 5 秒、最长绿 20 秒、每 2 秒决策，沿用相同黄灯和清空时间。
逐帧记录期望与实际灯态；换相允许最多 10 帧确认，未确认帧不计入配时。
稳定阶段失配立即停止。输出保留 APPLYING 帧，审计不把它们伪装成已生效阶段。

背景车使用 BasicAgent 跟随逐车道 OpenDRIVE 路线，不使用本机缺失的 TM.set_route。
适配器通过已验证的车道归属筛选本车信号，修复 0.9.12 BasicAgent 只匹配 road_id
导致邻车道红灯阻挡本车的问题。保留车辆障碍检测。车辆通过目标出口后退休，避免
在终点停车阻挡后续车流。当前需求是每回合有限批次，不是持续到达车流。

```bash
bash DriveSignalLab/integrations/cross3/run.sh \
  --controller fixed --background-per-lane 1 --duration 180 \
  --output /home/zlj/zlj/Drive/outputs/cross3/my_fixed_run
```

## 两辆 LMDrive

两辆独立 ego 来自西、东进口直行车道（road 1/3，lane -2），默认另有 12 辆背景车，
包含全部四向的左、直、右车流。真实 RGB/LiDAR/IMU/GNSS 输入和模型输出审计由
原 MDrive LMDrive 实现提供；没有用 BasicAgent 控制覆盖 LMDrive 验证输出。
消息关闭。复用既有停车预测 PID 数值保护，不改变正常前向预测的控制。

```bash
# 原始模型对照，不加载旧 Town05 轨迹头。
bash DriveSignalLab/integrations/cross3/run.sh \
  --egos 2 --controller fixed --duration 180 --seed 2027 \
  --output /home/zlj/zlj/Drive/outputs/cross3/my_raw_two_ego

# 显式选择本地图适配头和新地图信号策略。
bash DriveSignalLab/integrations/cross3/run.sh \
  --egos 2 --controller presslight --seed 2027 \
  --checkpoint /home/zlj/zlj/Drive/models/presslight/cross3_carla_seed2026.pt \
  --waypoint-head /home/zlj/zlj/Drive/models/lmdrive/cross3_waypoint_head.pt \
  --output /home/zlj/zlj/Drive/outputs/cross3/my_adapted_two_ego
```

`--teacher --egos 2` 是 BasicAgent 专家采集，**不能计为 LMDrive 成功**。
采集器保留模型特征和真实未来位姿，`fit_waypoint_head.py` 仅拟合最后一层。
使用这种头的结果属于新地图场景适配，不代表原始模型的未见地图泛化。

## 真实 CARLA PressLight 风格 DQN 训练

这是独立的 PressLight 风格实现，不是官方 PressLight 仓库复现。
状态 28 维：四相位 one-hot + 12 车道入口计数 + 12 对应出口计数；动作是四个相位。
奖励为实际 CARLA 入口/出口绝对压力和的负时间积分；经验回放、目标网络和 DQN 更新
使用真实世界轨迹，包含黄灯与清空阶段的时间折扣。固定右转不作为额外动作。
保存和读取模型时校验地图哈希、相位/车道顺序、维度、右转策略与安全时间设置。
旧的两动作/四进口测试权重不兼容。

建议训练使用独立服务端端口 31610，避免改变双车验证世界：

```bash
Drive/software/CARLA_0.9.12_MDrive/CarlaUE4.sh \
  --world-port=31610 -quality-level=Low -graphicsadapter=2 -RenderOffScreen

bash DriveSignalLab/integrations/cross3/run.sh \
  --port 31610 --controller presslight --train --episodes 100 \
  --background-per-lane 2 --duration 180 \
  --checkpoint /home/zlj/zlj/Drive/models/presslight/cross3_carla_seed2026.pt \
  --output /home/zlj/zlj/Drive/outputs/cross3/my_training_continuation
```

存在检查点时恢复网络、目标网络、优化器和计数器；经验回放在进程内保持，重启后重新积累。
每回合保存检查点及 JSON 元数据。未加 `--train` 时关闭探索，不更新权重。
首批训练通过只证明运行链路有效，不证明收敛、持续车流能力或优于固定配时。

## 审计与回放

```bash
python3 DriveSignalLab/integrations/cross3/audit_cross3.py Drive/outputs/cross3/my_fixed_run
python3 DriveSignalLab/integrations/cross3/build_replay.py \
  Drive/outputs/cross3/my_fixed_run/episode_000 \
  --output Drive/outputs/cross3/my_replay.html
```

验收要求车辆全部到达清单规定出口、没有错误出口/车道记录、没有碰撞回调、稳定灯态零失配；
双 LMDrive 验证还要求两个 ego 都有真实模型输出。车辆运行状态 `finished` 不等于验收通过。
出口目标为目标道路 `s=85 m`，退休判定为 `s<=88 m`（距目标约 3 m 内），并要求出口道路与
车道均正确。信号审计还独立核对四相位对应的车道集合，不能仅靠“期望值等于实际值”放行。
这不是官方路线评分，也没有完整闯红灯认证，不能报告 score_route=100 或零闯红灯。

已完成证据：

- `Drive/outputs/cross3/fixed_lane_signal_seed2026/audit.json`：12/12 到达目标出口，1997 帧一致，碰撞回调 0。
- `Drive/outputs/cross3/train_carla_confirmed_seed2026/audit.json`：12 回合、288/288 车到达目标出口，碰撞回调 0，稳定灯态失配 0。
- `Drive/models/presslight/cross3_carla_seed2026.json`：首批累计 496 次梯度更新。
- `Drive/outputs/cross3/presslight_greedy_seed2027/audit.json`：冻结检查点独立推理，24/24 到达指定出口。
- `Drive/outputs/cross3/two_lmdrive_fixed_raw_seed2026/`：原始 LMDrive 新地图对照存在出口车道偏离，主动停止；保留失败结果。
- `Drive/outputs/cross3/fixed_background_replay.html`：固定配时实测轨迹的交互式道路示意回放。

## 双 LMDrive + PressLight 闭环验收通过

运行目录：`Drive/outputs/cross3/two_lmdrive_presslight_adapted_seed2027/`。
使用 seed 2026 的专家数据适配输出层，1497 个样本；闭环验证 seed 2027，主车初始位置另有
按种子生成的小扰动。视觉编码器、QFormer、语言模型与轨迹头第一层保持原权重。
没有专家控制、地图转向覆盖或发车助推。

- 两辆 ego 均有真实 LMDrive 模型输出，分别约 44.3、49.4 仿真秒到达目标出口车道。
- 两辆主车加 12 辆背景车共 14/14 完成，背景车约 119.75 仿真秒排空。
- 四相位均放行，碰撞回调 0，出口/车道错误 0，稳定灯态失配 0，信号确认延迟帧 0。
- 控制器为冻结的 `cross3_carla_seed2026.pt`；本轮没有更新信号权重。
- 模型头为显式加载的 `cross3_waypoint_head.pt`，属于场景适配结果。
- 审计：运行目录下 `audit.json`，`accepted=true` 且 `vla_accepted=true`。
- 交互回放：`Drive/outputs/cross3/two_lmdrive_presslight_replay.html`。

同一 24 辆有限批次的初步对照，固定配时约 102.8 秒排空、累计采样排队 559.35 车秒；
首批 DQN 冻结推理约 122.8 秒、678.60 车秒。当前 DQN 尚无收益证据。
详见 `Drive/outputs/cross3/initial_training_comparison.json`。需要后续多需求、多种子、
持续到达训练与匹配评估，不能用工程链路通过代替策略有效性结论。

离线审计回归测试（3 项）：

```bash
python3 -m unittest discover -s DriveSignalLab/integrations/cross3 -p 'test_*.py' -v
```
