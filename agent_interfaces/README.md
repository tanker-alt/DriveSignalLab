# 灯控与智驾模型统一接入接口 v1

本模块定义两个独立的模型边界：`SignalAgent` 选择目标绿灯相位；`DrivingAgent` 消费传感器和路线并输出逐车控制量。两个插件可以独立替换，通过同一个 JSON 加载机制组合运行。模型不负责推进 CARLA 时钟。

## 入口与兼容范围

| 入口 | 灯控接入 | 智驾接入 | 用途 |
| --- | --- | --- | --- |
| `integrations/cross3/run_cross3.py` | `--signal-agent JSON` | `--driving-agent JSON` | Cross3 联合推理，支持 0 或 2 辆 ego |
| `integrations/cross3/official/evaluate_plugin.py` | `--signal-agent JSON` | 背景车仍为 BasicAgent | 使用官方实验环境、80 维观测与清空互锁评估新灯控 |
| `integrations/cross3/official/train.py` | 原官方 PressLight 训练流程 | 无 VLA | 保留 v4 训练协议，不通过插件修改训练算法 |

不传插件参数时，旧 Cross3 的固定配时、MaxPressure、PressLight-style DQN、LMDrive 和教师模式继续沿用。插件接口当前覆盖推理与评估；其他模型的训练、回放缓存、优化器与训练进程由模型自己的工具负责。v4 报告和历史实验结果没有因接口改造而重新验证。

## 配置格式

```json
{
  "schema": "drive_agent_plugin_v1",
  "kind": "signal",
  "factory": "my_package.signal_adapter:MySignalAgent",
  "kwargs": {"checkpoint": "/absolute/path/to/weights"}
}
```

`kind` 为 `signal` 或 `driving`；`factory` 为可导入 Python 模块和工厂函数/类；`kwargs` 的字符串支持环境变量与用户目录展开，然后传入构造器。插件配置执行本地 Python，须使用可信配置。插件包需安装在当前运行环境或加入 `PYTHONPATH`。路径参数建议全部写绝对路径；相对模型路径按进程工作目录解析，不按 JSON 所在目录解析。框架不自动安装依赖、下载权重或切换虚拟环境。示例所需环境变量见 [团队共享说明](../docs/CODE_SHARING.md)。

联合入口将 JSON 副本写入输出目录，便于记录所用配置。模型版本、权重哈希及输入预处理版本应由适配器自行记录；配置副本本身不代表权重已固定。

## 灯控契约

继承 `agent_interfaces.SignalAgent`，或提供相同的 `reset(context)`、`act(observation)`、`close()` 方法。

- `reset(context)`：每个 episode 开始时调用。`schema=signal_context_v1`，包含 `phase_order` 和 `observation_schema`。在这里校验模型支持的地图/特征/动作顺序，重置有记忆的状态。
- `act(observation)`：返回目标绿灯的整数索引，不接受布尔值、越界值、浮点数或灯态字典。
- `close()`：整个运行结束或失败后释放模型、线程、进程等资源。构造器失败时须自行释放已分配资源。

当前 Cross3 相位固定为 `0=NS_STRAIGHT, 1=NS_LEFT, 2=EW_STRAIGHT, 3=EW_LEFT`。它们是相位组，不是 12 条车道各一个动作。新模型如果使用另一种编号，必须在适配器内转换。

```python
from agent_interfaces import SignalAgent

class MySignalAgent(SignalAgent):
    name = 'my_signal_model'

    def reset(self, context):
        if context['observation_schema'] != 'cross3_counts_v1':
            raise ValueError('Unsupported observation schema')
        self.phase_order = context['phase_order']

    def act(self, observation):
        # Replace this example with model inference and action-index conversion.
        return max(range(len(observation['pressure'])),
                   key=observation['pressure'].__getitem__)
```

观测包 `schema=signal_observation_v1` 包含 `observation_schema`、`phase_order`、`current_action`、`pressure`、`elapsed_seconds`、`raw`。`current_action` 在初始全红时可以是 `None`；时间为仿真开始后的秒数，不能用墙钟时间替代。

| observation_schema | raw 内容 | 注意 |
| --- | --- | --- |
| `cross3_counts_v1` | `incoming/outgoing/pressure` 四相位统计，以及 `lane_incoming/lane_outgoing/lane_queued/elapsed` 原始字段 | 可供 Cross3 联合运行使用；不是官方 PressLight 特征 |
| `cross3_official_80_v1` | `state` 为官方环境原始状态，另有 `pressure/elapsed_seconds` | 8 位相位 + 36 维入口分箱 + 36 维出口分箱，由官方 worker 消费 |

不同观测 schema 不可直接互换，适配器应明确拒绝不支持的 schema。`SignalSelector` 桥接旧 `select(observation,current_action)` 调用点；`OfficialSignalCallback` 桥接官方环境的 `select(state,pressures,current,t)`。已有 selector 也可用 `SelectorAgent(selector)` 包装。

黄灯、全红、最小/最大绿灯和物理灯态由各环境继续执行，插件只返回目标相位。联合入口沿用 `SafeAdaptivePolicy`；官方评估入口沿用 v4 的车辆及车尾清空互锁。两套安全策略和决策频率并不完全相同，接口统一不意味着实验协议等价。最大绿灯机制可能覆盖模型请求，训练时不能把请求动作直接当成最终执行动作。

## 智驾契约

`agent_interfaces.DrivingAgent` 描述结构契约，不强制继承。推荐从当前安装的 MDrive `leaderboard.autoagents.autonomous_agent.AutonomousAgent` 继承，在适配器内部加载任意 VLA、规划模型或远程模型客户端。

工厂必须返回已初始化的 agent，提供 `ego_vehicles_num`、`sensor_interface`、字符串 `agent_name` 及以下方法：

| 方法 | 约定 |
| --- | --- |
| `sensors()` | 返回当前 MDrive AgentWrapper 支持的传感器规格；采用其逐 ego 传感器 ID 规范 |
| `set_global_plan(gps_routes, world_routes)` | 每辆车一条路线，顺序与 ego 索引一致；世界路线包含 Transform 和 RoadOption |
| `run_step(input_data, timestamp)` | 根据 SensorInterface 的同步观测，返回按 ego 顺序排列的 `carla.VehicleControl` 列表/元组 |
| `__call__()` | 由 MDrive 基类获取同步输入并调用 run_step；运行器通过 AgentWrapper 调用 |
| `destroy()` | 释放模型资源；传感器由 wrapper 清理 |

`input_data` 沿用 MDrive 的 `sensor_id -> (frame, value)` 数据格式，不经过 JSON 序列化；具体图像通道、点云格式、坐标系、单位和传感器 ID 应按所声明的传感器与 MDrive 版本在适配器内转换。不要将其他 Leaderboard 分支的单车 agent 直接当作本项目的多车 agent。

若模型输出轨迹，适配器必须完成坐标/单位转换和轨迹跟踪，最终输出 VehicleControl；框架不假定所有模型都输出相同轨迹格式。输出应满足 `throttle/brake` 在 `[0,1]`，`steer` 在 `[-1,1]`，且为有限数。运行器在应用控制前检查数量和数值，错误会停止运行并进入清理流程，不自动替换为其他模型。

适配器须支持运行器写入 `town_id='OpenDriveMap'`，并遵循 MDrive 多车传感器生命周期。当前联合入口只支持 2 辆 ego、单进程单 episode；需要改变车数或采用不同模拟器时还需扩展场景运行器。

新 VLA 的推荐适配结构如下；`my_model_package` 为你要接入的模型包，模型输出控制量的转换逻辑由该包提供：

```python
from leaderboard.autoagents.autonomous_agent import AutonomousAgent
from my_model_package import load_model

class MyDrivingAgent(AutonomousAgent):
    def setup(self, path_to_conf_file, ego_vehicles_num):
        self.ego_vehicles_num = ego_vehicles_num
        self.agent_name = 'my_vla'
        self.model = load_model(path_to_conf_file)

    def sensors(self):
        return self.model.mdrive_sensor_specs()

    def run_step(self, input_data, timestamp):
        return self.model.predict_vehicle_controls(
            input_data, timestamp, self._global_plan_world_coord_all)

    def destroy(self):
        self.model.close()
```

该示例继承当前 MDrive 基类的 `__call__`、SensorInterface 和多车 `set_global_plan`；模型包须实现示例中的三个模型方法，并按 ego 顺序输出控制。工厂 JSON 可直接指向该适配类，kwargs 使用 `path_to_conf_file` 和 `ego_vehicles_num`。

## 使用方式

以下命令从 `/home/zlj/zlj/DriveSignalLab` 执行，使用已配置 CARLA/MDrive 的 Python 环境；CARLA 需已启动在相应端口，地图和 overlay 沿用现有安装。

仅替换灯控，保留原 LMDrive 加载和运行环境设置：

```bash
python integrations/cross3/run_cross3.py \
  --port 31600 --egos 2 \
  --signal-agent agent_interfaces/examples/maxpressure.json \
  --output /tmp/cross3_signal_plugin_run
```

同时替换两者：

```bash
python integrations/cross3/run_cross3.py \
  --port 31600 --egos 2 \
  --signal-agent /absolute/path/my_signal.json \
  --driving-agent /absolute/path/my_driving.json \
  --output /tmp/cross3_two_plugins_run
```

`examples/lmdrive.json` 使用 `agent_interfaces.lmdrive:create_agent` 工厂，自动沿用旧入口的 LMDrive 专用环境设置（离线标志、日志路径、路线标识、关闭灯控文本通知、清除 waypoint head），并在导入模型前切换到 MDrive 工作目录。修改示例中的模型配置与日志输出路径后即可传给 `--driving-agent`。不加载 LMDrive 时不会导入其 torch/模型依赖。替换为其他 VLA 时由其工厂负责对应运行环境。

灯控插件优先于 `--controller`，不可与 `--train/--checkpoint` 合用；智驾插件不可与 `--teacher/--waypoint-head` 合用。教师仍为专用采集模式。

在官方环境中评估灯控插件：

```bash
python integrations/cross3/official/evaluate_plugin.py \
  --port 31610 --signal-agent agent_interfaces/examples/maxpressure.json \
  --seed 8101 --demand balanced --output /tmp/official_signal_plugin_run
```

将配置替换为 `examples/official_presslight.json` 可通过隔离的官方 Python worker 加载 v4 的 `round_059`，只做推理。先核对示例中的 Python、models、checkpoint、log 路径；checkpoint 是官方保存名称，不含 `.h5` 后缀。该适配器只接受官方 80 维观测，不能放到普通 Cross3 counts 入口。若要联合运行官方 PressLight 与 VLA，需要先为联合场景提供同语义的 80 维观测采集，不能只换 JSON 绕过此限制。

## 验证

```bash
python -m unittest discover -s agent_interfaces/tests -v
```

测试不依赖 CARLA 或模型权重，覆盖插件加载、观测桥接、动作校验、已有安全层连接、控制量校验和加载失败清理。真实模型接入后仍需在 CARLA 中检查传感器帧、路线顺序、控制输出、灯态切换和退出清理，再进行同需求、同随机种子、多种子的效果比较。
