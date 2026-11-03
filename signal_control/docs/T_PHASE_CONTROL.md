# 三叉路口逐进口相位控制

入口：`signal_control/control_t_phases.py`。默认地图为 `Maps/MapDataset/t3_north`。

每个进口最右侧车道常绿，包括最右侧为直行的车道；其余车道按三个进口依次放行。每相位默认绿灯 12 秒、黄灯 3 秒、受控车道清空至少 2 秒。清空阶段叫 `MAIN_RED`，此时最右侧车道仍然常绿。启动先真正全红至少 2 秒，退出恢复接管前灯色和配时。

清空互锁根据受控进口车辆的位置，以及车头、车尾（额外 0.5 米裕量）对应的原生路口 waypoint 判断占用；受控车辆尚未离开路口时延长清空，不放行下一进口。常绿车道的车辆独立跟踪，不使连续常绿车流阻断相位轮转。额外等待超过 `--max-clearance-wait`（默认 60 秒）则报错结束控制并执行退出恢复；不强制放行下一相位。日志的 `clearance_vehicle_ids` 保存每帧占用情况。

相位顺序随缺失方向旋转：缺北为西→南→东，缺东为北→西→南，缺南为东→北→西，缺西为南→东→北。

方向从所选路口的 `junctions[].arms` 推导，不使用地图级 `config.missing_arm`。对于当前右侧通行、独立车道信号的 OpenDRIVE 地图，最右侧进口车道按绝对值最大的车道编号确定。拒绝共享车道信号、缺失覆盖和非三叉路口。

在项目根目录运行（需 CARLA Python 环境）：

```powershell
python -B signal_control/control_t_phases.py --plan-only
# 先使用现有 view.py 加载对应地图，再运行相位控制。
python -B signal_control/control_t_phases.py --cycles 1
# 路网内选择一个三叉路口：
python -B signal_control/control_t_phases.py --map-dir Maps/MapDataset/t_1x6 --junction-id 10 --plan-only
```

正常运行不加载地图、不改变世界同步设置、不设置 TM 种子、不调用全局冻结。异步世界等待仿真帧；已有同步世界可以等待外部唯一 tick 所有者，或使用 `--tick` 自己作为唯一推进者。多个控制器不能同时使用 `--tick`。不要在控制运行期间调用 TM 随机种子重置接口。

`build_plan` 和 `expected_states` 是独立于 CARLA 的可复用接口，供后续路网统一调度使用。当前命令行一次只控制一个路口。接管前核验地图指纹、原生信号 ID、车道归属；日志逐帧记录期望和实测灯态，命令最多允许 10 帧生效延迟，稳定后失配立即停止。

集成动态车流时使用 `run(args, plan, sample_callback, client=client)`，让控制器与 Traffic Manager 共用同一个客户端；一批实验全程保留同一个客户端与 TM 实例，避免旧实例析构干扰下一用例。`sample_callback(snapshot, phase_index, stage, expected, actual)` 在每个仿真帧调用，不能额外推进仿真。

已完成离线相位规则和路网选择验证，以及 `t3_north` 的 CARLA 六车功能测试。复现入口为 `signal_control/validate_t_phases_live.py`，需预先加载地图且无其他实验车辆/传感器。

本次记录：`LocalArtifacts/t_phase_control/20261102_144817/report.json`。两轮相位循环，2099 帧、约 104.9 仿真秒；灯态失配 0，最右车道常绿失配 0，6/6 车到达预期出口车道，碰撞回调 0。南、东进口受控左转车辆记录到红灯停车。测试车辆和传感器均清理，恢复异步世界。

该单路口测试为单种子、每车道一车的功能测试。红灯过线检查使用车中心与原生信号 s 位置，尚不是车头越过标定停止线的正式违规指标。

路网方向与混合车流测试入口：`signal_control/validate_t_network_live.py`。先加载 `Maps/MapDataset/t_3x3`，默认选择缺西 10、缺东 10010、缺北 40010、缺南 70010 四个路口，分别运行低/中/高三档波次及 42/73 两个种子。每车道计划 1/3/6 辆，波次间隔为 12/6/2 秒，实际生成受入口空位约束，因此这些档位不等同于固定 veh/h 的饱和流量。六种车型为 Tesla Model 3、Nissan Patrol、Mercedes Sprinter、Mitsubishi Fuso Rosa、Carla Cola 卡车、Audi A2。车辆通过目标出口 30 米后退出测试并移到离路位置，统一在用例结束时销毁；每次只运行一个路口，不是九路口同时协调实验。

测试日志可使用 `signal_control/audit_t_network_validation.py <输出目录>` 独立审计。审计直接根据地图车道、缺失方向推导预期灯色，检查帧连续性、相位顺序、阶段最低时长、每次切绿前受控车尾已清空及实验现场恢复。

2026-11-02 完整矩阵结果为 **23/24 组通过，整体未通过**。缺南路口 70010 的高流量、种子 73 存在长车出口僵持，清空互锁超时报错。详见 [测试报告](T_NETWORK_VALIDATION_2026-11-02.md)；此问题解决前不能宣称四种方向的混合车流测试全部通过。
