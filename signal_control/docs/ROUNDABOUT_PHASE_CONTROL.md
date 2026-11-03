# 环岛相位控制

入口：`signal_control/control_roundabout_phases.py`。

控制器使用地图旁的 `phase_example.json` 作为相位计划。当前 A/B/C/D 与示意图一致：

- A：灯组 5、6、7、8，绿灯 24 秒
- B：灯组 2、4、6、8，绿灯 36 秒
- C：灯组 5、6、7、8，绿灯 24 秒
- D：灯组 1、3、5、7，绿灯 36 秒

每个相位都可以独立设置 `green_seconds`、`yellow_seconds`、`all_red_seconds`；缺省时使用计划文件中的全局黄灯/全红值。命令行 `--green`、`--yellow`、`--all-red` 可临时覆盖所有相位，便于短时灯态回归测试。

示例：

```powershell
# 使用地图默认 A/B/C/D 配时运行一轮
python -B signal_control/control_roundabout_phases.py --cycles 1

# 用自定义计划运行；每相位可独立配时
python -B signal_control/control_roundabout_phases.py --phase-config Maps/MapDataset/roundabout4_8groups/phase_example.json --cycles 2

# 短时灯态读回测试：统一覆盖为 1 秒
python -B signal_control/control_roundabout_phases.py --cycles 1 --green 1 --yellow 1 --all-red 1
```

启动前校验 OpenDRIVE 指纹、原生信号 ID、每盏灯的车道归属和灯组覆盖；每帧核验期望灯态与 CARLA 实际灯态。相位切换顺序为绿灯、黄灯、全红。退出时恢复原配时、灯色、冻结状态和世界设置。

该脚本默认只控制信号灯，不生成车辆、不启用美化、不做交通容量或冲突安全验收。`lane_phase_candidate.json` 仍是静态候选冲突检查结果，不能替代车辆实测。
