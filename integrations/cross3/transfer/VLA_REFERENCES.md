# 联合 VLA 退化复查与参考实现

本轮方法为自定义 PPO 原型，不是 OpenVLA/OFT/RLinf 官方训练配方复现。
真实 LMDrive 冻结上游；在内部末层 4096→10 线性层上建立固定方差高斯策略；
实际轨迹经原 PID 执行。灯控与车辆共享全体车辆耗时/碰撞奖励、同一时间段 GAE 优势。
灯控学习率 3e-4；VLA 内部层 1e-5，PPO clip 0.2，VLA 梯度裁剪 0.1。

## 由保存的真实 rollout 复算

在 `outputs/staged/joint_small_noise/train/run_00{0,1}/episode_000/rollout.pt` 中取
实际 VLA 输入特征与采样均值，用本回合更新后的 checkpoint 内部层重新前向。
以 `sum((new_mean-old_mean)^2)/(2*std^2)` 计算固定方差高斯 KL（std=0.005 m）：

| 回合 | 样本 | 平均KL | 最大KL | 10维输出差 RMS/m | 最大绝对输出差/m |
|---|---:|---:|---:|---:|---:|
| 1 | 2143 | 8.04530 | 33.97367 | 0.00634244 | 0.01336956 |
| 2 | 2021 | 4.53281 | 17.31352 | 0.00476068 | 0.01263771 |

代码阈值为 0.03，但检查在优化器步之前，之后超限只 break，不回滚已执行的步。
因此该实现不是有效的更新后 KL 上界保护。较大的 KL 是相对于很小的探索方差，
不表示绝对轨迹一定偏移数米，也不单独证明错误出口的唯一成因。

进一步代码风险：一秒内多次推理的对数概率比累计后取 exp，却按单次推理平均KL检查，
保护尺度不一致；两车共享路口级优势；critic 不包含完整驾驶视觉/路线误差；
没有专家行为克隆损失，仅当前特征上对初始化输出做均方锚定。错误出口直接抛异常，
失败回合未作为有明确路线失败惩罚的完整 episode 参与训练。以上均需修正后单独验证。

## 已在线核对的公开来源

- LMDrive: Closed-Loop End-to-End Driving with Large Language Models, CVPR 2024.
  https://arxiv.org/abs/2312.07488
  https://github.com/opendilab/LMDrive
  最贴近现有模型：核对原训练数据、坐标、轨迹监督与闭环评估。不是现成联合灯控RL。
- OpenVLA: An Open-Source Vision-Language-Action Model.
  https://arxiv.org/abs/2406.09246
  https://github.com/openvla/openvla
  官方提供 LoRA/full/partial 微调与故障排查；核心任务是机器人操作。
- Fine-Tuning Vision-Language-Action Models: Optimizing Speed and Success (OpenVLA-OFT).
  https://arxiv.org/abs/2502.19645
  https://github.com/moojink/openvla-oft
  LIBERO 文档提供 LoRA r=32、连续动作 L1 回归与动作块训练配置。
  不是现有 LMDrive 最后一层高斯 PPO 的直接替代代码。
- RLinf-VLA: A Unified and Efficient Framework for VLA+RL Training.
  https://arxiv.org/abs/2510.06710
  https://github.com/RLinf/RLinf
  https://rlinf.readthedocs.io/en/latest/rst_source/resources/publications/rlinf_vla.html
  提供 OpenVLA/OpenVLA-OFT 的 PPO/GRPO 参考；文档中的稳定性结论来自其机器人基准，
  不应外推成 CARLA 必然改善。
- OpenPI (π0/π0.5)
  https://github.com/Physical-Intelligence/openpi
  自有数据集微调、归一化与 LoRA 配置的工程参考；机器人动作/架构不等于驾驶轨迹。

建议：优先按 LMDrive 原监督接口做目标场景的轨迹适配，保留正确驾驶回放，
证明冻结灯控下 VLA 不退化；再采用有更新后KL检查/回滚、驾驶级奖励和参考策略约束的
RL 微调，最后逐步开启灯控共同更新。LoRA 是参数化手段，不会自动修复奖励和信用分配。
